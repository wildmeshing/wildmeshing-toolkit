#pragma once
#include <Eigen/SVD>
#include <bit>
#include <cassert>
#include <unordered_set>
#include <wmtk/utils/Logger.hpp>
#include "types.hpp"
namespace wmtk::components::prismatic_mesh::quality {
using native::Cell;
using native::Edge;
using native::Key;
using native::M;
using native::Mesh;
using native::V;
struct DSU
{
    std::vector<size_t> p;
    explicit DSU(size_t n)
        : p(n)
    {
        std::iota(p.begin(), p.end(), 0);
    }
    size_t get(size_t a) { return p[a] == a ? a : p[a] = get(p[a]); }
    void join(size_t a, size_t b)
    {
        a = get(a);
        b = get(b);
        if (a != b) p[a] = b;
    }
};
struct Face
{
    size_t a = size_t(-1), b = size_t(-1);
    std::vector<size_t> v;
};
using Faces = std::unordered_map<Key, Face, native::ArrayHash<5>>;
inline Faces faces_of(const std::vector<Cell>& cells)
{
    Faces out;
    out.reserve(cells.size() * 2 + 100);
    for (size_t i = 0; i < cells.size(); ++i)
        for (const auto& local : native::faces.at(cells[i].type)) {
            std::vector<size_t> v;
            for (int j : local) v.push_back(cells[i].v[j]);
            auto& f = out[native::facekey(v)];
            if (f.a == size_t(-1)) {
                f.a = i;
                f.v = native::cycle(v);
            } else if (f.b == size_t(-1)) {
                auto reverse = v;
                std::reverse(reverse.begin(), reverse.end());
                if (native::cycle(reverse) != f.v) throw std::runtime_error("face_orientation");
                f.b = i;
            } else
                throw std::runtime_error("nonmanifold_volume_face");
        }
    return out;
}
inline std::map<Key, std::vector<size_t>> cell_boundary(const std::vector<Cell>& cc)
{
    std::map<Key, std::vector<size_t>> out;
    for (const auto& [key, f] : faces_of(cc))
        if (f.b == size_t(-1)) out[key] = f.v;
    return out;
}
inline Json topology(const Mesh& m, const Faces& ff)
{
    DSU cells(m.cells.size()), surface(m.p.size());
    std::vector<std::vector<size_t>> interface;
    std::set<std::pair<int, int>> adjacency;
    for (const auto& [key, f] : ff)
        if (f.b != size_t(-1)) {
            int a = m.cells[f.a].region, b = m.cells[f.b].region;
            if (a == b)
                cells.join(f.a, f.b);
            else {
                adjacency.insert(std::minmax(a, b));
                if ((a == 2 && b == 3) || (a == 3 && b == 2)) {
                    interface.push_back(f.v);
                    for (size_t v : f.v) surface.join(f.v[0], v);
                }
            }
        }
    std::array<std::set<size_t>, 4> components;
    for (size_t i = 0; i < m.cells.size(); ++i) components[m.cells[i].region].insert(cells.get(i));
    struct Part
    {
        std::set<size_t> v;
        std::set<Edge> e;
        size_t f = 0;
    };
    std::map<size_t, Part> parts;
    for (const auto& f : interface) {
        auto& p = parts[surface.get(f[0])];
        ++p.f;
        for (size_t j = 0; j < f.size(); ++j) {
            p.v.insert(f[j]);
            p.e.insert(native::edge(f[j], f[(j + 1) % f.size()]));
        }
    }
    std::vector<int64_t> euler;
    for (const auto& [id, p] : parts)
        euler.push_back(int64_t(p.v.size()) - int64_t(p.e.size()) + int64_t(p.f));
    std::sort(euler.begin(), euler.end());
    return {
        {"region_face_connected_components",
         {components[1].size(), components[2].size(), components[3].size()}},
        {"shell_background_interface_component_Euler", euler},
        {"region_adjacencies", adjacency}};
}
inline Json quality(const Mesh& m)
{
    Json j = Json::object();
    for (int r : {1, 2, 3})
        for (int t : {4, 6, 7}) {
            size_t n = 0, b10 = 0, b20 = 0, b100 = 0, b1000 = 0;
            double max = 0;
            long double sum = 0;
            for (const auto& c : m.cells)
                if (c.region == r && c.type == t) {
                    ++n;
                    b10 += c.energy > 10;
                    b20 += c.energy > 20;
                    b100 += c.energy > 100;
                    b1000 += c.energy > 1000;
                    max = std::max(max, c.energy);
                    sum += c.energy;
                }
            if (n)
                j[std::to_string(r) + "_" + std::to_string(t)] = {
                    {"count", n},
                    {"maximum", max},
                    {"mean", double(sum / n)},
                    {"over10", b10},
                    {"over20", b20},
                    {"over100", b100},
                    {"over1000", b1000}};
        }
    return j;
}
struct InputFace
{
    std::array<size_t, 3> v;
    int kind = 0;
    size_t patch = 0;
};
struct Rules
{
    std::map<Key, InputFace> kept;
    std::set<Key> released, dynamically_released;
    std::vector<V> patch_normal, patch_origin;
    std::vector<double> patch_area;
    std::vector<uint8_t> originally_input;
    std::vector<V> original_positions;
    Json initial_counts;
    Rules() = default;
    explicit Rules(const Mesh& m, const Faces& ff)
        : originally_input(m.input)
        , original_positions(m.p)
    {
        std::vector<InputFace> list;
        std::array<size_t, 4> counts{};
        for (const auto& [key, f] : ff) {
            if (key[0] != 3) continue;
            bool input = true;
            for (size_t v : f.v) input &= bool(m.input[v]);
            if (!input) continue;
            std::vector<size_t> sides;
            if (m.cells[f.a].region == 2) sides.push_back(f.a);
            if (f.b != size_t(-1) && m.cells[f.b].region == 2) sides.push_back(f.b);
            if (sides.empty()) continue;
            int kind = 3;
            if (sides.size() == 2 && m.cells[sides[0]].type == 4 && m.cells[sides[1]].type == 4)
                kind = 0;
            else if (sides.size() == 1 && m.cells[sides[0]].type == 4)
                kind = 1;
            else if (
                sides.size() == 2 && (m.cells[sides[0]].type == 4 || m.cells[sides[1]].type == 4))
                kind = 2;
            ++counts[kind];
            if (!kind) {
                released.insert(key);
                continue;
            }
            list.push_back({{f.v[0], f.v[1], f.v[2]}, kind, 0});
        }
        DSU dsu(list.size());
        std::map<Edge, std::vector<size_t>> edge_faces;
        std::vector<V> normals;
        for (size_t i = 0; i < list.size(); ++i) {
            const auto& v = list[i].v;
            V n = (m.p[v[1]] - m.p[v[0]]).cross(m.p[v[2]] - m.p[v[0]]).normalized();
            Eigen::Index d;
            n.cwiseAbs().maxCoeff(&d);
            if (n[d] < 0) n = -n;
            normals.push_back(n);
            for (int j = 0; j < 3; ++j) edge_faces[native::edge(v[j], v[(j + 1) % 3])].push_back(i);
        }
        for (const auto& [e, fs] : edge_faces)
            if (fs.size() == 2) {
                size_t a = fs[0], b = fs[1];
                bool coplanar = (normals[a] - normals[b]).norm() < 1e-10;
                for (size_t v : list[b].v)
                    coplanar &= std::abs(normals[a].dot(m.p[v] - m.p[list[a].v[0]])) < 1e-10;
                if (coplanar) dsu.join(a, b);
            }
        std::map<size_t, size_t> first;
        std::set<size_t> nonplanar;
        for (size_t i = 0; i < list.size(); ++i) {
            size_t root = dsu.get(i);
            if (!first.count(root)) first[root] = i;
            size_t ref = first[root];
            for (size_t v : list[i].v)
                if (std::abs(normals[ref].dot(m.p[v] - m.p[list[ref].v[0]])) > 1e-10)
                    nonplanar.insert(root);
        }
        std::map<size_t, size_t> ids;
        for (size_t i = 0; i < list.size(); ++i) {
            size_t root = dsu.get(i);
            if (nonplanar.count(root)) root = list.size() + i;
            if (!ids.count(root)) {
                ids[root] = ids.size();
                patch_normal.push_back(normals[i]);
                patch_origin.push_back(m.p[list[i].v[0]]);
                patch_area.push_back(0);
            }
            auto face = list[i];
            face.patch = ids[root];
            orient(m, face);
            patch_area[face.patch] += area(m, face);
            kept.emplace(native::facekey({face.v[0], face.v[1], face.v[2]}), face);
        }
        initial_counts = {
            {"released_two_shell_tets", counts[0]},
            {"one_sided_shell_tet_feature_surface", counts[1]},
            {"tet_opposite_native", counts[2]},
            {"native_only_input", counts[3]},
            {"coplanar_feature_patches", patch_area.size()}};
    }
    void orient(const Mesh& m, InputFace& f) const
    {
        if ((m.p[f.v[1]] - m.p[f.v[0]])
                .cross(m.p[f.v[2]] - m.p[f.v[0]])
                .dot(patch_normal[f.patch]) < 0)
            std::swap(f.v[1], f.v[2]);
    }
    double area(const Mesh& m, const InputFace& f) const
    {
        return .5 * (m.p[f.v[1]] - m.p[f.v[0]])
                        .cross(m.p[f.v[2]] - m.p[f.v[0]])
                        .dot(patch_normal[f.patch]);
    }
    size_t update_cases(const Mesh& m)
    {
        auto ff = faces_of(m.cells);
        size_t removed = 0;
        for (auto it = kept.begin(); it != kept.end();) {
            auto f = ff.find(it->first);
            if (f == ff.end())
                throw std::runtime_error("tracked_input_face_missing_before_case_update");
            const auto& c = f->second;
            if (c.b != size_t(-1) && m.cells[c.a].region == 2 && m.cells[c.b].region == 2 &&
                m.cells[c.a].type == 4 && m.cells[c.b].type == 4) {
                patch_area[it->second.patch] -= area(m, it->second);
                dynamically_released.insert(it->first);
                it = kept.erase(it);
                ++removed;
            } else
                ++it;
        }
        return removed;
    }
    void refresh_roles(Mesh& m) const
    {
        m.input.assign(m.p.size(), false);
        for (const auto& [k, f] : kept)
            for (size_t v : f.v) m.input[v] = true;
        std::vector<uint8_t> native_point(m.p.size(), false);
        for (const auto& c : m.cells)
            if (c.type != 4)
                for (size_t v : c.v) native_point[v] = true;
        for (size_t v = 0; v < originally_input.size(); ++v)
            if (native_point[v] && originally_input[v]) m.input[v] = true;
        std::vector<uint8_t> interface(m.p.size(), false);
        auto ff = faces_of(m.cells);
        for (const auto& [k, f] : ff)
            if (f.b != size_t(-1) && m.cells[f.a].region != m.cells[f.b].region)
                for (size_t v : f.v) interface[v] = true;
        for (size_t v = 0; v < m.p.size(); ++v) {
            if (m.fixed(v)) continue;
            if (m.input[v]) {
                m.correspondence[v] = -1;
                m.thickness[v] = 0;
                continue;
            }
            if (native_point[v]) {
                if (m.correspondence[v] >= 0 && !m.input.at(m.correspondence[v]))
                    m.correspondence[v] = -3;
                continue;
            }
            if (!interface[v] && !m.outer[v]) {
                m.correspondence[v] = -1;
                m.thickness[v] = NAN;
                m.base[v] = m.initial[v] = m.p[v];
            } else if (m.correspondence[v] >= 0)
                m.correspondence[v] = -3;
        }
    }
    void split_labels(const Mesh& m, const std::map<Edge, size_t>& mid)
    {
        auto old = kept;
        kept.clear();
        for (const auto& [key, f] : old) {
            std::vector<InputFace> children{f};
            for (const auto& [e, v] : mid) {
                std::vector<InputFace> next;
                for (auto child : children) {
                    auto a = std::find(child.v.begin(), child.v.end(), e[0]),
                         b = std::find(child.v.begin(), child.v.end(), e[1]);
                    if (a == child.v.end() || b == child.v.end()) {
                        next.push_back(child);
                        continue;
                    }
                    auto x = child, y = child;
                    x.v[a - child.v.begin()] = v;
                    y.v[b - child.v.begin()] = v;
                    orient(m, x);
                    orient(m, y);
                    next.push_back(x);
                    next.push_back(y);
                }
                children = std::move(next);
            }
            for (const auto& child : children)
                if (!kept.emplace(native::facekey({child.v[0], child.v[1], child.v[2]}), child)
                         .second)
                    throw std::runtime_error("duplicate_protected_input_face");
        }
    }
    Json audit(const Mesh& m, const Faces& ff) const
    {
        std::vector<long double> coverage(patch_area.size());
        double plane = 0;
        for (const auto& [key, f] : kept) {
            if (!ff.count(key))
                throw std::runtime_error(
                    "protected_input_face_lost:" +
                    Json({{"face", f.v}, {"kind", f.kind}, {"patch", f.patch}}).dump());
            for (size_t v : f.v)
                plane = std::max(
                    plane,
                    std::abs(patch_normal[f.patch].dot(m.p[v] - patch_origin[f.patch])));
            const double a = area(m, f);
            if (!(a > 0)) throw std::runtime_error("input_face_flip");
            coverage[f.patch] += a;
        }
        if (plane > 1e-9) throw std::runtime_error("input_geometry_moved:" + Json(plane).dump());
        double error = 0;
        for (size_t i = 0; i < coverage.size(); ++i) {
            if (patch_area[i] > 1e-12)
                error =
                    std::max(error, double(std::abs(coverage[i] - patch_area[i]) / patch_area[i]));
            else if (std::abs(coverage[i]) > 1e-10)
                throw std::runtime_error("released_input_patch_still_covered");
        }
        if (error > 1e-8) throw std::runtime_error("input_patch_coverage_changed");
        size_t absent = 0;
        for (const auto& k : released) absent += !ff.count(k);
        return {
            {"protected_faces", kept.size()},
            {"maximum_plane_error", plane},
            {"maximum_relative_patch_area_error", error},
            {"released_original_faces_now_absent", absent},
            {"released_original_faces_total", released.size()},
            {"new_tet_tet_faces_released_after_native_subdivision", dynamically_released.size()}};
    }
};
} // namespace wmtk::components::prismatic_mesh::quality
