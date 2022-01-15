#include "HarmonicTet.hpp"

#include <igl/predicates/predicates.h>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/utils/io.hpp>

#include <queue>
#include <wmtk/ExecutionScheduler.hpp>
#include "wmtk/utils/EnergyHarmonicTet.hpp"

namespace harmonic_tet {

double HarmonicTet::get_quality(const Tuple& loc)
{
    Eigen::MatrixXd ps(4, 3);
    auto tups = oriented_tet_vertices(loc);
    for (auto j = 0; j < 4; j++) {
        ps.row(j) = m_vertex_attribute[tups[j].vid(*this)];
    }
    return wmtk::harmonic_energy(ps);
}


bool HarmonicTet::is_inverted(const Tuple& loc)
{
    std::array<Eigen::Vector3d, 4> ps;
    auto tups = oriented_tet_vertices(loc);
    for (auto j = 0; j < 4; j++) {
        ps[j] = m_vertex_attribute[tups[j].vid(*this)];
    }

    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(ps[0], ps[1], ps[2], ps[3]);
    if (res == igl::predicates::Orientation::NEGATIVE) return false; // extremely annoying.
    return true;
}


bool harmonic_tet::HarmonicTet::smooth_after(const Tuple& t)
{
    wmtk::logger().trace("Gradient Descent iteration for vertex smoothing.");
    auto vid = t.vid(*this);

    auto locs = get_one_ring_tets_for_vertex(t);
    assert(locs.size() > 0);
    std::vector<std::array<double, 12>> assembles(locs.size());
    auto loc_id = 0;

    for (auto& loc : locs) {
        auto& T = assembles[loc_id];
        auto t_id = loc.tid(*this);

        assert(!is_inverted(loc));
        auto local_tuples = oriented_tet_vertices(loc);
        std::array<size_t, 4> local_verts;
        for (auto i = 0; i < 4; i++) {
            local_verts[i] = local_tuples[i].vid(*this);
        }

        local_verts = wmtk::orient_preserve_tet_reorder(local_verts, vid);

        for (auto i = 0; i < 4; i++) {
            for (auto j = 0; j < 3; j++) {
                T[i * 3 + j] = m_vertex_attribute[local_verts[i]][j];
            }
        }
        loc_id++;
    }

    auto old_pos = m_vertex_attribute[vid];
    m_vertex_attribute[vid] = wmtk::gradient_descent_from_stack(
        assembles,
        wmtk::harmonic_tet_energy,
        wmtk::harmonic_tet_jacobian);
    wmtk::logger().trace(
        "old pos {} -> new pos {}",
        old_pos.transpose(),
        m_vertex_attribute[vid].transpose());
    // note: duplicate code snippets.
    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            m_vertex_attribute[vid] = old_pos;
            return false;
        }
    }

    for (auto& loc : locs) {
        auto t_id = loc.tid(*this);
    }
    return true;
}

bool HarmonicTet::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(get_quality(l), max_energy);
    }
    edgeswap_cache.max_energy = max_energy;
    return true;
}
bool HarmonicTet::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) return false;

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    auto max_energy = std::max(get_quality(t), get_quality(*oppo_tet));
    wmtk::logger().debug("energy {} {}", edgeswap_cache.max_energy, max_energy);
    if (is_inverted(t) || is_inverted(*oppo_tet)) {
        wmtk::logger().debug("invert w/ energy {} {}", edgeswap_cache.max_energy, max_energy);
        return false;
    }
    if (max_energy > edgeswap_cache.max_energy) return false;
    return true;
}

bool HarmonicTet::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    for (auto& l : incident_tets) {
        if (is_inverted(l)) {
            return false;
        }
    }
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(get_quality(l), max_energy);
    }
    wmtk::logger().trace("quality {} from {}", max_energy, edgeswap_cache.max_energy);

    if (max_energy > edgeswap_cache.max_energy) return false;
    return true;
}

auto renewal = [](const auto& m, const auto& newt) {
    auto new_edges = std::vector<wmtk::TetMesh::Tuple>();
    assert(newt.switch_tetrahedron(m));
    for (auto ti : {newt.tid(m), newt.switch_tetrahedron(m)->tid(m)}) {
        for (auto j = 0; j < 6; j++) new_edges.push_back(m.tuple_from_edge(ti, j));
    };
    wmtk::unique_edge_tuples(m, new_edges);
    return new_edges;
};

void HarmonicTet::swap_all_edges()
{
    auto executor = wmtk::ExecutePass<HarmonicTet>();
    executor.renew_neighbor_tuples = renewal;
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();

    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_swap", loc);
    executor(*this, collect_all_ops);
}

void harmonic_tet::HarmonicTet::smooth_all_vertices()
{
    auto executor = wmtk::ExecutePass<HarmonicTet>();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_vertices()) collect_all_ops.emplace_back("vertex_smooth", loc);
    executor(*this, collect_all_ops);
}


void HarmonicTet::swap_all_faces()
{
    auto executor = wmtk::ExecutePass<HarmonicTet>();
    executor.renew_neighbor_tuples = renewal;
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_faces()) collect_all_ops.emplace_back("face_swap", loc);
    executor(*this, collect_all_ops);
};

bool HarmonicTet::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) return false;

    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    faceswap_cache.max_energy = std::max(get_quality(t), get_quality(*oppo_tet));
    return true;
}

void HarmonicTet::output_mesh(std::string file) const
{
    // warning: duplicate code.
    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return m_vertex_attribute[i];
    });

    const auto& tets = get_tets();
    msh.add_tets(tets.size(), [&](size_t k) {
        auto i = tets[k].tid(*this);
        auto vs = oriented_tet_vertices(tets[k]);
        std::array<size_t, 4> data;
        for (int j = 0; j < 4; j++) {
            data[j] = vs[j].vid(*this);
            assert(data[j] < vtx.size());
        }
        return data;
    });

    msh.add_tet_vertex_attribute<1>("tv index", [&](size_t i) { return i; });
    msh.add_tet_attribute<1>("t index", [&](size_t i) { return i; });

    msh.save(file, true);
}


} // namespace harmonic_tet