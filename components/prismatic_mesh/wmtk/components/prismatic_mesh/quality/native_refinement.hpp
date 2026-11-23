#pragma once
#include <fstream>
#include <nlohmann/json.hpp>

#include <deque>
#include <unordered_set>
#include "native_audit.hpp"
#include "thickness_feasibility.hpp"

namespace wmtk::components::prismatic_mesh::quality::native {
using Edge = std::array<size_t, 2>;
inline Edge edge(size_t a, size_t b)
{
    if (a > b) std::swap(a, b);
    return {a, b};
}
template <size_t N>
struct ArrayHash
{
    size_t operator()(const std::array<size_t, N>& a) const noexcept
    {
        size_t h = 0;
        for (size_t x : a)
            h ^= std::hash<size_t>{}(x) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        return h;
    }
};
inline std::vector<Edge> cell_edges(const Cell& c)
{
    std::set<Edge> out;
    for (const auto& f : faces.at(c.type))
        for (size_t j = 0; j < f.size(); ++j)
            out.insert(edge(c.v[f[j]], c.v[f[(j + 1) % f.size()]]));
    return {out.begin(), out.end()};
}
inline double cell_volume(const std::vector<V>& p, const Cell& c)
{
    if (c.type == 4) return determinant(p, c.v) / 6;
    if (c.type == 6) {
        long double sum = 0;
        for (const auto& q : quadrature(6, 2))
            sum += q.w * matrix(p, c, q).determinant() * std::sqrt(3.) / 4;
        return double(sum);
    }
    const auto& v = c.v;
    M a;
    const V p1 = p[v[1]] - p[v[0]], p2 = p[v[2]] - p[v[0]], p3 = p[v[3]] - p[v[0]];
    a.col(0) = (p1 + p2 - p3) / 2;
    a.col(1) = (-p1 + p2 + p3) / 2;
    a.col(2) = std::sqrt(2.) * (p[v[4]] - p[v[0]] - (p1 + p2 + p3) / 4);
    return a.determinant() / (3 * std::sqrt(2.));
}

struct PatchMetric
{
    std::array<double, 4> maximum{};
    long double volume = 0, weighted_sum = 0;
    double worst = 0;
    Json json() const
    {
        return {
            {"maximum_AMIPS", worst},
            {"region_maximum", maximum},
            {"volume_weighted_mean_AMIPS", double(weighted_sum / std::max(1e-300L, volume))}};
    }
};
inline PatchMetric patch_metric(const Mesh& m, const std::vector<Cell>& cells)
{
    PatchMetric q;
    for (const auto& c : cells) {
        const double v = cell_volume(m.p, c);
        q.volume += v;
        q.weighted_sum += v * c.energy;
        q.worst = std::max(q.worst, c.energy);
        q.maximum.at(c.region) = std::max(q.maximum.at(c.region), c.energy);
    }
    return q;
}
inline bool patch_quality_improves(const PatchMetric& before, const PatchMetric& after)
{
    for (int r = 1; r <= 3; ++r)
        if (after.maximum[r] > before.maximum[r] * (1 + 1e-10)) return false;
    return after.worst < before.worst * (1 - 1e-9) ||
           after.weighted_sum / std::max(1e-300L, after.volume) <
               before.weighted_sum / std::max(1e-300L, before.volume) * (1 - 1e-8);
}
inline bool offset_vertex(const Mesh& m, size_t v)
{
    return !m.input[v] && !m.outer[v] && m.correspondence[v] >= 0 &&
           std::isfinite(m.thickness[v]) && m.thickness[v] > 0;
}

// Native mixed-cell, atomic refinement. A cap-edge cut propagates through
// the prism's opposite cap; a column cut propagates through all 3 columns.
// Unsupported combinations are rejected before any state is changed.
class ConformingRefiner
{
    struct FaceEntry
    {
        size_t cell;
        std::vector<size_t> oriented;
    };
    Mesh& mesh;
    std::vector<uint8_t> live;
    std::vector<size_t> root;
    std::vector<double> original_volumes;
    std::vector<V> original_points;
    std::unordered_map<Edge, std::set<size_t>, ArrayHash<2>> edges;
    std::unordered_map<Key, std::vector<FaceEntry>, ArrayHash<5>> face_index;
    std::set<Edge> input_edges;
    std::map<Edge, std::set<Edge>> column_edges_by_input;
    size_t accepted = 0;
    bool column_parent_edge(const Edge& e, Edge& parent) const
    {
        const int64_t a = mesh.input[e[0]] ? int64_t(e[0]) : mesh.correspondence[e[0]],
                      b = mesh.input[e[1]] ? int64_t(e[1]) : mesh.correspondence[e[1]];
        if (a < 0 || b < 0 || a == b) return false;
        parent = edge(size_t(a), size_t(b));
        return true;
    }
    bool needs_column_reference(const Edge& e) const
    {
        const auto it = edges.find(e);
        if (it == edges.end()) return false;
        bool shell = false, background = false;
        for (size_t ci : it->second) {
            const auto& c = mesh.cells[ci];
            if (c.type != 4) return true;
            shell |= c.region == 2;
            background |= c.region == 3;
            for (const auto& f : faces.at(c.type)) {
                std::vector<size_t> fv;
                for (int j : f) fv.push_back(c.v[j]);
                if (std::find(fv.begin(), fv.end(), e[0]) != fv.end() &&
                    std::find(fv.begin(), fv.end(), e[1]) != fv.end() &&
                    face_index.at(facekey(fv)).size() == 1)
                    return true;
            }
        }
        return shell && background;
    }

    bool pure_tet_column_edge(Edge e) const
    {
        if (mesh.input[e[0]] || mesh.input[e[1]]) return false;
        auto it = edges.find(e);
        if (it == edges.end()) return false;
        bool shell = false;
        for (size_t i : it->second) {
            const auto& c = mesh.cells[i];
            if (c.type != 4 || (c.region != 2 && c.region != 3)) return false;
            shell |= c.region == 2;
        }
        return shell;
    }

    // An input edge may already have been subdivided while a pure-tet offset
    // edge with those original parents still exists. Reuse its input midpoint
    // instead of demanding an edge that no longer exists or duplicating it.
    int64_t existing_input_midpoint(Edge parent) const
    {
        const V a = mesh.p[parent[0]], d = mesh.p[parent[1]] - a, mid = (a + mesh.p[parent[1]]) / 2;
        const double l2 = d.squaredNorm();
        if (!(l2 > 0)) return -1;
        int64_t result = -1;
        for (size_t v = 0; v < mesh.p.size(); ++v)
            if (mesh.input[v] && mesh.p[v] == mid) {
                if (result >= 0) return -1;
                result = int64_t(v);
            }
        if (result < 0) return -1;
        std::map<size_t, std::vector<size_t>> graph;
        auto on_segment = [&](size_t v) {
            const V x = mesh.p[v] - a;
            const double t = x.dot(d) / l2;
            return t >= -1e-12 && t <= 1 + 1e-12 && x.cross(d).squaredNorm() <= 1e-24 * l2 * l2;
        };
        for (const auto& e : input_edges)
            if (edges.count(e) && on_segment(e[0]) && on_segment(e[1])) {
                graph[e[0]].push_back(e[1]);
                graph[e[1]].push_back(e[0]);
            }
        std::set<size_t> seen{parent[0]};
        std::vector<size_t> todo{parent[0]};
        while (!todo.empty()) {
            const size_t v = todo.back();
            todo.pop_back();
            for (size_t w : graph[v])
                if (seen.insert(w).second) todo.push_back(w);
        }
        return seen.count(parent[1]) && seen.count(size_t(result)) ? result : -1;
    }

    void add(size_t i)
    {
        const auto& c = mesh.cells[i];
        for (const auto& e : cell_edges(c)) {
            edges[e].insert(i);
            Edge parent;
            if (c.region == 2 && column_parent_edge(e, parent))
                column_edges_by_input[parent].insert(e);
        }
        for (const auto& f : faces.at(c.type)) {
            std::vector<size_t> v;
            bool all_input = true;
            for (int j : f) {
                v.push_back(c.v[j]);
                all_input &= bool(mesh.input[c.v[j]]);
            }
            face_index[facekey(v)].push_back({i, cycle(v)});
            if (all_input)
                for (size_t j = 0; j < v.size(); ++j)
                    input_edges.insert(edge(v[j], v[(j + 1) % v.size()]));
        }
    }
    void remove(size_t i)
    {
        const auto& c = mesh.cells[i];
        live[i] = false;
        for (const auto& e : cell_edges(c)) {
            auto it = edges.find(e);
            it->second.erase(i);
            if (it->second.empty()) {
                Edge parent;
                if (column_parent_edge(e, parent)) {
                    auto p = column_edges_by_input.find(parent);
                    if (p != column_edges_by_input.end()) {
                        p->second.erase(e);
                        if (p->second.empty()) column_edges_by_input.erase(p);
                    }
                }
                edges.erase(it);
            }
        }
        for (const auto& f : faces.at(c.type)) {
            std::vector<size_t> v;
            for (int j : f) v.push_back(c.v[j]);
            auto it = face_index.find(facekey(v));
            auto& es = it->second;
            es.erase(
                std::remove_if(es.begin(), es.end(), [&](const auto& e) { return e.cell == i; }),
                es.end());
            if (es.empty()) face_index.erase(it);
        }
    }
    void resize_points(size_t n)
    {
        mesh.p.resize(n);
        mesh.initial.resize(n);
        mesh.base.resize(n);
        mesh.thickness.resize(n);
        mesh.radius.resize(n);
        mesh.sizing.resize(n);
        mesh.input.resize(n);
        mesh.outer.resize(n);
        mesh.correspondence.resize(n);
    }

public:
    const auto& active_edges() const { return edges; }
    bool midpoint_is_offset(const Edge& e) const { return needs_column_reference(e); }
    bool is_input_edge(const Edge& e) const { return input_edges.count(e) > 0; }
    bool is_input_surface_edge(const Edge& e) const
    {
        const auto it = edges.find(e);
        if (it == edges.end() || !input_edges.count(e)) return false;
        for (size_t ci : it->second)
            for (const auto& f : faces.at(mesh.cells[ci].type)) {
                std::vector<size_t> fv;
                bool input = true;
                for (int j : f) {
                    const size_t v = mesh.cells[ci].v[j];
                    fv.push_back(v);
                    input = input && mesh.input[v] != 0;
                }
                if (!input || std::find(fv.begin(), fv.end(), e[0]) == fv.end() ||
                    std::find(fv.begin(), fv.end(), e[1]) == fv.end())
                    continue;
                const auto& es = face_index.at(facekey(fv));
                bool volume = false, outside = false;
                for (const auto& owner : es) {
                    volume |= mesh.cells[owner.cell].region == 1;
                    outside |= mesh.cells[owner.cell].region != 1;
                }
                if (volume && (outside || es.size() == 1)) return true;
            }
        return false;
    }
    Json events = Json::array(), rejections = Json::object();
    std::vector<Edge> point_parents;
    explicit ConformingRefiner(Mesh& m)
        : mesh(m)
        , live(m.cells.size(), true)
        , original_points(m.p)
    {
        root.resize(m.cells.size());
        std::iota(root.begin(), root.end(), 0);
        for (size_t v = 0; v < m.p.size(); ++v) point_parents.push_back({v, v});
        for (size_t i = 0; i < m.cells.size(); ++i) {
            original_volumes.push_back(cell_volume(m.p, m.cells[i]));
            add(i);
        }
    }
    struct Plan
    {
        std::set<Edge> marks;
        std::set<size_t> patch;
        std::string reason = "ok";
    };
    Plan closure(
        Edge start,
        bool allow_input_split = true,
        size_t max_cells = 20000,
        bool interpolated_tet_columns = false,
        bool paired_column_splits = false) const
    {
        Plan plan;
        plan.marks.insert(start);
        std::deque<Edge> queue{start};
        while (!queue.empty()) {
            const auto e = queue.front();
            queue.pop_front();
            const auto found = edges.find(e);
            if (found == edges.end()) {
                plan.reason = "missing_edge";
                return plan;
            }
            if (!allow_input_split && input_edges.count(e)) {
                plan.reason = "input_edge";
                return plan;
            }
            if (paired_column_splits) {
                Edge parent;
                if (column_parent_edge(e, parent)) {
                    if (input_edges.count(parent) && edges.count(parent)) {
                        if (plan.marks.insert(parent).second) queue.push_back(parent);
                    } else if (existing_input_midpoint(parent) < 0) {
                        plan.reason = "paired_input_parent_edge_missing";
                        return plan;
                    }
                    if (auto it = column_edges_by_input.find(parent);
                        it != column_edges_by_input.end())
                        for (const auto& x : it->second)
                            if (edges.count(x) && plan.marks.insert(x).second) queue.push_back(x);
                }
            }
            if (!mesh.input[e[0]] && !mesh.input[e[1]] && mesh.correspondence[e[0]] >= 0 &&
                mesh.correspondence[e[1]] >= 0 &&
                mesh.correspondence[e[0]] != mesh.correspondence[e[1]]) {
                if (interpolated_tet_columns && pure_tet_column_edge(e)) {
                    // No native cap is being cut: reference interpolation does
                    // not require an unrelated input-surface topology change.
                } else {
                    const auto parent =
                        edge(size_t(mesh.correspondence[e[0]]), size_t(mesh.correspondence[e[1]]));
                    if (!input_edges.count(parent) || !edges.count(parent)) {
                        if (existing_input_midpoint(parent) < 0) {
                            plan.reason = "correspondence_parent_edge_missing";
                            return plan;
                        }
                    } else if (plan.marks.insert(parent).second)
                        queue.push_back(parent);
                }
            }
            for (size_t i : found->second) {
                plan.patch.insert(i);
                if (plan.patch.size() > max_cells) {
                    plan.reason = "closure_budget";
                    return plan;
                }
                const auto& c = mesh.cells[i];
                std::vector<Edge> extra;
                if (c.type == 6) {
                    const int a = int(std::find(c.v.begin(), c.v.end(), e[0]) - c.v.begin());
                    const int b = int(std::find(c.v.begin(), c.v.end(), e[1]) - c.v.begin());
                    if (a % 3 == b % 3)
                        for (int j = 0; j < 3; ++j) extra.push_back(edge(c.v[j], c.v[j + 3]));
                    else {
                        extra.push_back(edge(c.v[a % 3], c.v[b % 3]));
                        extra.push_back(edge(c.v[a % 3 + 3], c.v[b % 3 + 3]));
                    }
                } else if (c.type == 7 && e[0] != c.v[4] && e[1] != c.v[4]) {
                    for (int j = 0; j < 4; ++j)
                        if (edge(c.v[j], c.v[(j + 1) % 4]) == e)
                            extra.push_back(edge(c.v[(j + 2) % 4], c.v[(j + 3) % 4]));
                }
                for (const auto& x : extra)
                    if (plan.marks.insert(x).second) queue.push_back(x);
            }
        }
        for (size_t i : plan.patch) {
            const auto& c = mesh.cells[i];
            size_t marked = 0, lateral = 0, vertical = 0;
            for (const auto& e : cell_edges(c))
                if (plan.marks.count(e)) {
                    ++marked;
                    if (c.type == 7) lateral += e[0] == c.v[4] || e[1] == c.v[4];
                    if (c.type == 6)
                        for (int j = 0; j < 3; ++j) vertical += e == edge(c.v[j], c.v[j + 3]);
                }
            if (c.type == 6 &&
                !((marked == 3 && vertical == 3) || (marked == 2 && vertical == 0))) {
                plan.reason = "prism_pattern_conflict";
                return plan;
            }
            if (c.type == 7 && !((marked == 1 && lateral == 1) || (marked == 2 && lateral == 0))) {
                plan.reason = "pyramid_pattern_conflict";
                return plan;
            }
        }
        return plan;
    }
    bool split(
        Edge start,
        bool allow_input_split = true,
        size_t max_cells = 20000,
        bool enforce_patch_volume = true,
        bool quality_gate = false,
        bool interpolated_tet_columns = false,
        double maximum_input_amips = std::numeric_limits<double>::infinity(),
        bool paired_column_splits = false)
    {
        const auto plan = closure(
            start,
            allow_input_split,
            max_cells,
            interpolated_tet_columns,
            paired_column_splits);
        auto reject = [&](const std::string& reason) {
            rejections[reason] = rejections.value(reason, size_t(0)) + 1;
            return false;
        };
        if (plan.reason != "ok") return reject(plan.reason);
        const size_t old_n = mesh.p.size();
        std::map<Edge, size_t> mid;
        std::map<size_t, Edge> parents;
        for (const auto& e : plan.marks) {
            const size_t v = mesh.p.size();
            mid[e] = v;
            parents[v] = e;
            point_parents.push_back(e);
            const V p = (mesh.p[e[0]] + mesh.p[e[1]]) / 2,
                    b = (mesh.base[e[0]] + mesh.base[e[1]]) / 2;
            const bool is_input = input_edges.count(e) > 0;
            const V reference = (mesh.initial[e[0]] + mesh.initial[e[1]]) / 2;
            mesh.p.push_back(p);
            mesh.initial.push_back(reference);
            mesh.base.push_back(is_input ? p : b);
            const bool referenced =
                std::isfinite(mesh.thickness[e[0]]) && std::isfinite(mesh.thickness[e[1]]);
            mesh.thickness.push_back(
                is_input     ? 0
                : referenced ? (reference - b).norm()
                             : std::numeric_limits<double>::quiet_NaN());
            mesh.radius.push_back(0);
            mesh.sizing.push_back((mesh.sizing[e[0]] + mesh.sizing[e[1]]) / 2);
            mesh.input.push_back(is_input);
            bool outer = false;
            for (size_t ci : edges.at(e))
                for (const auto& f : faces.at(mesh.cells[ci].type)) {
                    std::vector<size_t> fv;
                    for (int j : f) fv.push_back(mesh.cells[ci].v[j]);
                    if (std::find(fv.begin(), fv.end(), e[0]) != fv.end() &&
                        std::find(fv.begin(), fv.end(), e[1]) != fv.end() &&
                        face_index.at(facekey(fv)).size() == 1)
                        outer = true;
                }
            mesh.outer.push_back(outer);
            mesh.correspondence.push_back(-1);
        }
        for (const auto& [e, v] : mid)
            if (!mesh.input[v]) {
                if (paired_column_splits && !needs_column_reference(e)) {
                    mesh.correspondence[v] = -1;
                    mesh.thickness[v] = std::numeric_limits<double>::quiet_NaN();
                    mesh.base[v] = mesh.initial[v] = mesh.p[v];
                    continue;
                }
                const int64_t a = mesh.input[e[0]] ? int64_t(e[0]) : mesh.correspondence[e[0]];
                const int64_t b = mesh.input[e[1]] ? int64_t(e[1]) : mesh.correspondence[e[1]];
                if (a >= 0 && b >= 0) {
                    if (a == b)
                        mesh.correspondence[v] = a;
                    else if (auto it = mid.find(edge(size_t(a), size_t(b)));
                             it != mid.end() && mesh.input[it->second])
                        mesh.correspondence[v] = it->second;
                    else
                        mesh.correspondence[v] =
                            existing_input_midpoint(edge(size_t(a), size_t(b)));
                }
                // A cross-column edge inside a tet region creates a free volume
                // point, not an offset point with a fabricated column reference.
                // Boundary/interface use still has to pass the global validator.
                if (mesh.correspondence[v] < 0 && interpolated_tet_columns &&
                    pure_tet_column_edge(e) && std::isfinite(mesh.thickness[e[0]]) &&
                    mesh.thickness[e[0]] > 0 && std::isfinite(mesh.thickness[e[1]]) &&
                    mesh.thickness[e[1]] > 0)
                    mesh.correspondence[v] = -2;
                if (mesh.correspondence[v] < 0 && mesh.correspondence[v] != -2) {
                    mesh.thickness[v] = std::numeric_limits<double>::quiet_NaN();
                    mesh.base[v] = mesh.initial[v] = mesh.p[v];
                }
            }
        auto rollback = [&](const std::string& reason) {
            resize_points(old_n);
            point_parents.resize(old_n);
            return reject(reason);
        };
        std::vector<Cell> children;
        std::vector<size_t> child_roots;
        Json old_types = Json::object(), new_types = Json::object();
        double max_before = 0, max_after = 0;
        long double volume_before = 0, volume_after = 0;
        for (size_t i : plan.patch) {
            const auto& c = mesh.cells[i];
            const auto& v = c.v;
            old_types[std::to_string(c.type)] =
                old_types.value(std::to_string(c.type), size_t(0)) + 1;
            max_before = std::max(max_before, c.energy);
            volume_before += cell_volume(mesh.p, c);
            std::vector<Cell> out;
            auto child = [&](int type, std::vector<size_t> vertices) {
                Cell x;
                x.type = type;
                x.region = c.region;
                x.v = std::move(vertices);
                out.push_back(std::move(x));
            };
            if (c.type == 4) {
                std::vector<std::vector<size_t>> cells{v};
                for (const auto& e : plan.marks) {
                    if (std::find(v.begin(), v.end(), e[0]) == v.end() ||
                        std::find(v.begin(), v.end(), e[1]) == v.end())
                        continue;
                    std::vector<std::vector<size_t>> next;
                    for (const auto& t : cells) {
                        const auto a = std::find(t.begin(), t.end(), e[0]),
                                   b = std::find(t.begin(), t.end(), e[1]);
                        if (a == t.end() || b == t.end()) {
                            next.push_back(t);
                            continue;
                        }
                        auto x = t, y = t;
                        x[a - t.begin()] = mid.at(e);
                        y[b - t.begin()] = mid.at(e);
                        next.push_back(x);
                        next.push_back(y);
                    }
                    cells = std::move(next);
                }
                for (auto& t : cells) child(4, std::move(t));
            } else if (c.type == 6) {
                if (plan.marks.count(edge(v[0], v[3]))) {
                    const size_t a = mid.at(edge(v[0], v[3])), b = mid.at(edge(v[1], v[4])),
                                 d = mid.at(edge(v[2], v[5]));
                    child(6, {v[0], v[1], v[2], a, b, d});
                    child(6, {a, b, d, v[3], v[4], v[5]});
                } else {
                    int j = 0;
                    while (!plan.marks.count(edge(v[j], v[(j + 1) % 3]))) ++j;
                    const int a = j, b = (j + 1) % 3, d = (j + 2) % 3;
                    const size_t m = mid.at(edge(v[a], v[b])), n = mid.at(edge(v[a + 3], v[b + 3]));
                    child(6, {v[a], m, v[d], v[a + 3], n, v[d + 3]});
                    child(6, {m, v[b], v[d], n, v[b + 3], v[d + 3]});
                }
            } else {
                int j = -1;
                for (int k = 0; k < 4; ++k)
                    if (plan.marks.count(edge(v[k], v[4]))) j = k;
                if (j >= 0) {
                    const size_t a = v[j], b = v[(j + 1) % 4], d = v[(j + 2) % 4],
                                 f = v[(j + 3) % 4], ap = v[4], m = mid.at(edge(a, ap));
                    child(7, {a, b, d, f, m});
                    child(4, {ap, m, b, d});
                    child(4, {ap, m, d, f});
                } else {
                    j = 0;
                    while (!plan.marks.count(edge(v[j], v[(j + 1) % 4]))) ++j;
                    const size_t a = v[j], b = v[(j + 1) % 4], d = v[(j + 2) % 4],
                                 f = v[(j + 3) % 4], ap = v[4];
                    const size_t m = mid.at(edge(a, b)), n = mid.at(edge(d, f));
                    child(7, {a, m, n, f, ap});
                    child(7, {m, b, d, n, ap});
                }
            }
            for (auto& x : out) {
                if (x.type == 4 && determinant(mesh.p, x.v) < 0) std::swap(x.v[0], x.v[1]);
                if (!std::isfinite(quality(mesh.p, x))) return rollback("nonpositive_float_child");
                if (x.region == 1 && x.energy > maximum_input_amips)
                    return rollback("input_child_over_limit");
                if (quality_gate && c.energy <= 100 && x.energy > 100)
                    return rollback("formerly_good_child_exceeds_100");
                if (quality_gate && x.region == 2 && !thickness_cell_feasible(mesh, x, -1e-8))
                    return rollback("continuous_thickness");
                max_after = std::max(max_after, x.energy);
                volume_after += cell_volume(mesh.p, x);
                new_types[std::to_string(x.type)] =
                    new_types.value(std::to_string(x.type), size_t(0)) + 1;
                children.push_back(std::move(x));
                child_roots.push_back(root[i]);
            }
        }
        std::vector<Cell> old_cells;
        for (size_t i : plan.patch) old_cells.push_back(mesh.cells[i]);
        const auto old_metric = patch_metric(mesh, old_cells),
                   new_metric = patch_metric(mesh, children);
        if (quality_gate && !patch_quality_improves(old_metric, new_metric))
            return rollback("patch_quality_not_improved");
        if (enforce_patch_volume && std::abs(volume_after - volume_before) >
                                        1e-10L * std::max(1e-16L, std::abs(volume_before)))
            return rollback("patch_volume");
        std::unordered_map<Key, std::vector<std::vector<size_t>>, ArrayHash<5>> replacement;
        std::unordered_set<Key, ArrayHash<5>> touched;
        std::vector<std::vector<size_t>> exterior_before;
        for (size_t i : plan.patch)
            for (const auto& f : faces.at(mesh.cells[i].type)) {
                std::vector<size_t> v;
                for (int j : f) v.push_back(mesh.cells[i].v[j]);
                const auto key = facekey(v);
                touched.insert(key);
                if (face_index.at(key).size() == 1) exterior_before.push_back(v);
            }
        for (const auto& c : children)
            for (const auto& f : faces.at(c.type)) {
                std::vector<size_t> v;
                for (int j : f) v.push_back(c.v[j]);
                const auto key = facekey(v);
                touched.insert(key);
                replacement[key].push_back(cycle(v));
            }
        for (const auto& key : touched) {
            auto entries = replacement[key];
            if (auto it = face_index.find(key); it != face_index.end())
                for (const auto& e : it->second)
                    if (!plan.patch.count(e.cell)) entries.push_back(e.oriented);
            if (entries.size() > 2) return rollback("nonmanifold_face");
            if (entries.size() == 2) {
                auto reverse = entries[1];
                std::reverse(reverse.begin(), reverse.end());
                if (entries[0] != cycle(reverse)) return rollback("face_orientation");
            }
            if (entries.size() == 1) {
                bool permitted = false;
                for (const auto& old_face : exterior_before) {
                    bool subset = true;
                    for (size_t v : entries[0]) {
                        if (std::find(old_face.begin(), old_face.end(), v) != old_face.end())
                            continue;
                        const auto p = parents.find(v);
                        if (p == parents.end() ||
                            std::find(old_face.begin(), old_face.end(), p->second[0]) ==
                                old_face.end() ||
                            std::find(old_face.begin(), old_face.end(), p->second[1]) ==
                                old_face.end()) {
                            subset = false;
                            break;
                        }
                    }
                    if (subset) {
                        permitted = true;
                        break;
                    }
                }
                if (!permitted) return rollback("internal_crack");
            }
        }
        for (size_t i : plan.patch) remove(i);
        for (size_t j = 0; j < children.size(); ++j) {
            const size_t i = mesh.cells.size();
            mesh.cells.push_back(std::move(children[j]));
            root.push_back(child_roots[j]);
            live.push_back(true);
            add(i);
        }
        ++accepted;
        size_t input_new = 0;
        for (size_t v = old_n; v < mesh.p.size(); ++v) input_new += mesh.input[v];
        Json lineage = Json::array();
        for (const auto& [e, v] : mid)
            lineage.push_back(
                {{"vertex", v},
                 {"parents", e},
                 {"input", bool(mesh.input[v])},
                 {"correspondence_id", mesh.correspondence[v]}});
        events.push_back(
            {{"seed_edge", start},
             {"split_edges", plan.marks.size()},
             {"old_cells", plan.patch.size()},
             {"new_cells", children.size()},
             {"old_types", old_types},
             {"new_types", new_types},
             {"new_input_vertices", input_new},
             {"maximum_AMIPS_before", max_before},
             {"maximum_AMIPS_after", max_after},
             {"quality_gate", quality_gate},
             {"patch_quality_before", old_metric.json()},
             {"patch_quality_after", new_metric.json()},
             {"new_vertices", lineage}});
        return true;
    }
    // Root-cell volumes and original coordinates independently check that
    // refinement has not changed the pre-existing geometry or mixed material.
    Json validate() const
    {
        for (size_t v = 0; v < original_points.size(); ++v)
            if (mesh.p[v] != original_points[v])
                throw std::runtime_error("Refinement moved an original point.");
        for (size_t v = original_points.size(); v < mesh.p.size(); ++v)
            if (mesh.p[v] != (mesh.p[point_parents[v][0]] + mesh.p[point_parents[v][1]]) / 2)
                throw std::runtime_error("New vertex is not its edge midpoint.");
        std::vector<long double> volumes(original_volumes.size());
        size_t alive = 0, boundary = 0;
        for (size_t i = 0; i < live.size(); ++i)
            if (live[i]) {
                volumes[root[i]] += cell_volume(mesh.p, mesh.cells[i]);
                ++alive;
            }
        double max_abs = 0;
        long double total = 0, old_total = 0;
        for (size_t i = 0; i < volumes.size(); ++i) {
            max_abs = std::max(max_abs, double(std::abs(volumes[i] - original_volumes[i])));
            total += volumes[i];
            old_total += original_volumes[i];
        }
        for (const auto& [key, es] : face_index) {
            if (es.empty() || es.size() > 2)
                throw std::runtime_error("Invalid face incidence after refinement.");
            if (es.size() == 1)
                ++boundary;
            else {
                auto reverse = es[1].oriented;
                std::reverse(reverse.begin(), reverse.end());
                if (es[0].oriented != cycle(reverse))
                    throw std::runtime_error("Face orientation mismatch after refinement.");
            }
        }
        const double relative =
            double(std::abs(total - old_total) / std::max(1e-300L, std::abs(old_total)));
        if (relative > 1e-10 || max_abs > 1e-11)
            throw std::runtime_error("Original material-cell volumes changed.");
        return {
            {"accepted_atomic_splits", accepted},
            {"original_points_exactly_unchanged", true},
            {"active_cells", alive},
            {"boundary_faces", boundary},
            {"maximum_original_cell_volume_change", max_abs},
            {"relative_total_volume_change", relative},
            {"rejections", rejections}};
    }
    struct Candidate
    {
        Edge edge;
        double face_bound;
        int shell_type;
    };
    std::vector<Edge> due_background_boundary_edges(
        double splitting_l2,
        const std::set<Edge>& forced = {}) const
    {
        std::set<Edge> unique;
        for (const auto& [key, es] : face_index)
            if (key[0] == 3) {
                size_t bg = 0;
                for (const auto& e : es) bg += mesh.cells[e.cell].region == 3;
                if (bg != 1) continue;
                const auto& v = es[0].oriented;
                for (size_t j = 0; j < v.size(); ++j) {
                    const auto e = edge(v[j], v[(j + 1) % v.size()]);
                    const double s = (mesh.sizing[e[0]] + mesh.sizing[e[1]]) / 2;
                    if (forced.count(e) ||
                        (mesh.p[e[0]] - mesh.p[e[1]]).squaredNorm() >= splitting_l2 * s * s)
                        unique.insert(e);
                }
            }
        std::vector<Edge> result(unique.begin(), unique.end());
        std::sort(result.begin(), result.end(), [&](const auto& a, const auto& b) {
            const double x = (mesh.p[a[0]] - mesh.p[a[1]]).squaredNorm(),
                         y = (mesh.p[b[0]] - mesh.p[b[1]]).squaredNorm();
            return x == y ? a < b : x > y;
        });
        return result;
    }
    std::vector<Candidate> interface_candidates(double minimum_bound = 100, bool all_edges = false)
        const
    {
        std::map<Edge, Candidate> candidates;
        for (const auto& [key, es] : face_index)
            if (key[0] == 3 && es.size() == 2) {
                size_t bg = size_t(-1), shell_cell = size_t(-1);
                for (const auto& e : es) {
                    if (mesh.cells[e.cell].region == 3) bg = e.cell;
                    if (mesh.cells[e.cell].region == 2) shell_cell = e.cell;
                }
                if (bg == size_t(-1) || shell_cell == size_t(-1)) continue;
                const auto& v = es[0].oriented;
                const V a = mesh.p[v[1]] - mesh.p[v[0]], b = mesh.p[v[2]] - mesh.p[v[0]];
                const double area = a.cross(b).norm() / 2;
                double s = 0, longest = -1;
                Edge seed{};
                for (int j = 0; j < 3; ++j) {
                    const auto e = edge(v[j], v[(j + 1) % 3]);
                    const double l = (mesh.p[e[0]] - mesh.p[e[1]]).squaredNorm();
                    s += l;
                    if (l > longest) {
                        longest = l;
                        seed = e;
                    }
                }
                const double lower = std::pow(3 * s / (4 * area), 2. / 3);
                if (lower <= minimum_bound) continue;
                std::vector<Edge> seeds{seed};
                if (all_edges) {
                    seeds.clear();
                    for (int j = 0; j < 3; ++j) seeds.push_back(edge(v[j], v[(j + 1) % 3]));
                }
                for (const auto& e : seeds)
                    if (!candidates.count(e) || candidates.at(e).face_bound < lower)
                        candidates[e] = {e, lower, mesh.cells[shell_cell].type};
            }
        std::vector<Candidate> out;
        for (const auto& [e, c] : candidates) out.push_back(c);
        std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
            if (a.face_bound != b.face_bound) return a.face_bound > b.face_bound;
            return a.edge < b.edge;
        });
        return out;
    }
    void compact()
    {
        std::vector<Cell> cells;
        std::vector<size_t> roots;
        for (size_t i = 0; i < live.size(); ++i)
            if (live[i]) {
                cells.push_back(std::move(mesh.cells[i]));
                roots.push_back(root[i]);
            }
        mesh.cells = std::move(cells);
        root = std::move(roots);
        live.assign(mesh.cells.size(), true);
        edges.clear();
        face_index.clear();
        input_edges.clear();
        column_edges_by_input.clear();
        for (size_t i = 0; i < mesh.cells.size(); ++i) add(i);
        mesh.rebuild();
    }
    const std::vector<size_t>& owners() const { return root; }
};
} // namespace wmtk::components::prismatic_mesh::quality::native
