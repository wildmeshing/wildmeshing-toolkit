#include "volume_io.hpp"
#include "volume_geometry.hpp"

#include <wmtk/io/mesh_io.hpp>
#include <wmtk/utils/Logger.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace wmtk::components::tracking {

TetMeshData read_tet_msh(const std::string& path)
{
    // Through io::read_mesh, which reads the layout io::write_mesh writes -- every cell once, in
    // a block per set of physical groups, its element tag its row -- as well as the older one
    // that lists a tet once per group.
    MeshData msh;
    try {
        msh = io::read_mesh(path);
    } catch (const std::exception& e) {
        log_and_throw_error("Could not read {}: {}", path, e.what());
    }
    TetMeshData m;
    for (Eigen::Index i = 0; i < msh.V.rows(); ++i) {
        m.vertices.emplace_back(msh.V.row(i).transpose());
    }
    const auto block = msh.cells.find(CellType::Tetrahedron);
    if (block == msh.cells.end() || block->second.connectivity.rows() == 0) {
        log_and_throw_error("{} holds no tets", path);
    }
    const auto& T = block->second.connectivity;
    // The physical groups that list each row: the columns of its `tags`.
    const auto tags = block->second.attributes.find("tags");
    DataMatrix<int64_t> member;
    std::vector<std::string> labels;
    if (tags != block->second.attributes.end()) {
        member = tags->second.integers();
        labels = tags->second.labels;
    }
    std::map<std::array<uint64_t, 4>, size_t> index;
    std::vector<std::set<std::string>> groups;
    for (Eigen::Index r = 0; r < T.rows(); ++r) {
        std::array<uint64_t, 4> t;
        for (int k = 0; k < 4; ++k) t[k] = uint64_t(T(r, k));
        std::array<uint64_t, 4> key = t;
        std::sort(key.begin(), key.end());
        const auto [jt, fresh] = index.emplace(key, m.tets.size());
        if (fresh) {
            m.tets.push_back(t);
            groups.emplace_back();
        }
        for (size_t j = 0; j < labels.size(); ++j) {
            if (member(r, Eigen::Index(j)) != 0) groups[jt->second].insert(labels[j]);
        }
    }
    index.clear();
    std::map<std::set<std::string>, uint32_t> set_index;
    for (auto& g : groups) {
        const auto [it, fresh] = set_index.emplace(std::move(g), uint32_t(m.group_sets.size()));
        if (fresh) m.group_sets.push_back(it->first);
        m.groups.push_back(it->second);
    }
    return m;
}

TetLocator::TetLocator(const TetMeshData& m)
    : m_mesh(m)
{
    m_lo = m.vertices[0];
    Eigen::Vector3d hi = m_lo;
    for (const auto& v : m.vertices) {
        m_lo = m_lo.cwiseMin(v);
        hi = hi.cwiseMax(v);
    }
    const double cells = std::max(1., std::cbrt(double(m.tets.size()) / 4));
    m_h = std::max((hi - m_lo).maxCoeff() / cells, 1e-300);
    for (int k = 0; k < 3; ++k) m_n[k] = std::max<int64_t>(1, int64_t((hi[k] - m_lo[k]) / m_h) + 1);
    m_cells.resize(size_t(m_n[0] * m_n[1] * m_n[2]));
    const auto cell = [&](const Eigen::Vector3d& p, int k) {
        return std::clamp<int64_t>(int64_t((p[k] - m_lo[k]) / m_h), 0, m_n[k] - 1);
    };
    for (size_t t = 0; t < m.tets.size(); ++t) {
        Eigen::Vector3d a = m.vertices[m.tets[t][0]], b = a;
        for (int k = 0; k < 3; ++k) {
            const auto& v = m.vertices[m.tets[t][k + 1]];
            a = a.cwiseMin(v);
            b = b.cwiseMax(v);
        }
        for (int64_t x = cell(a, 0); x <= cell(b, 0); ++x) {
            for (int64_t y = cell(a, 1); y <= cell(b, 1); ++y) {
                for (int64_t z = cell(a, 2); z <= cell(b, 2); ++z) {
                    m_cells[size_t((x * m_n[1] + y) * m_n[2] + z)].push_back(uint32_t(t));
                }
            }
        }
    }
}

std::pair<int64_t, Eigen::Vector4d> TetLocator::locate(const Eigen::Vector3d& p) const
{
    return locate(p, uint32_t(-1));
}

std::pair<int64_t, Eigen::Vector4d> TetLocator::locate(
    const Eigen::Vector3d& p,
    const uint32_t group) const
{
    int64_t c[3];
    for (int k = 0; k < 3; ++k) {
        c[k] = int64_t(std::floor((p[k] - m_lo[k]) / m_h));
        if (c[k] < 0 || c[k] >= m_n[k]) return {-1, Eigen::Vector4d::Zero()};
    }
    // Exact containment, as the volume map locates its points (volume::locate_in): the tets
    // holding p, else those it is out of by a rounding; of the group first, if given.
    const auto& V = m_mesh.vertices;
    std::vector<uint32_t> holding, within_rounding;
    for (const uint32_t t : m_cells[size_t((c[0] * m_n[1] + c[1]) * m_n[2] + c[2])]) {
        const auto& T = m_mesh.tets[t];
        Eigen::Vector3d lo = V[T[0]], hi = lo;
        for (int k = 1; k < 4; ++k) {
            lo = lo.cwiseMin(V[T[k]]);
            hi = hi.cwiseMax(V[T[k]]);
        }
        const double eps = 1e-9 * (hi - lo).norm();
        if (((p.array() < lo.array() - eps) || (p.array() > hi.array() + eps)).any()) continue;
        const auto o = volume::outside_of(p, V[T[0]], V[T[1]], V[T[2]], V[T[3]]);
        if (o.faces == 0) {
            holding.push_back(t);
        } else if (o.distance <= eps) {
            within_rounding.push_back(t);
        }
    }
    const auto coordinates = [&](const uint32_t t, const bool exact) {
        const auto& T = m_mesh.tets[t];
        Eigen::Vector4d b = exact ? volume::exact_barycentric(p, V[T[0]], V[T[1]], V[T[2]], V[T[3]])
                                  : volume::barycentric(p, V[T[0]], V[T[1]], V[T[2]], V[T[3]]);
        if (!exact && !(b.minCoeff() >= -1e-9 && b.maxCoeff() <= 1 + 1e-9)) {
            b = volume::exact_barycentric(p, V[T[0]], V[T[1]], V[T[2]], V[T[3]]);
        }
        return b;
    };
    const auto clamped = [](Eigen::Vector4d b) {
        b = b.cwiseMax(0.);
        return Eigen::Vector4d(b / b.sum());
    };
    for (const bool own : {true, false}) {
        for (const uint32_t t : holding) {
            if (own && m_mesh.groups[t] != group) continue;
            return {int64_t(t), clamped(coordinates(t, false))};
        }
        std::pair<int64_t, Eigen::Vector4d> best{-1, Eigen::Vector4d::Zero()};
        double best_min = -std::numeric_limits<double>::infinity();
        for (const uint32_t t : within_rounding) {
            if (own && m_mesh.groups[t] != group) continue;
            const Eigen::Vector4d b = coordinates(t, true);
            if (b.minCoeff() > best_min) {
                best_min = b.minCoeff();
                best = {int64_t(t), b};
            }
        }
        if (best.first >= 0) return {best.first, clamped(best.second)};
    }
    return {-1, Eigen::Vector4d::Zero()};
}

bool TetLocator::in_group(const Eigen::Vector3d& p, const uint32_t group) const
{
    const auto [t, b] = locate(p, group);
    return t >= 0 && m_mesh.groups[size_t(t)] == group;
}

} // namespace wmtk::components::tracking
