#pragma once
#include "boundary_surface.hpp"
#include "common.hpp"
namespace wmtk::components::prismatic_mesh::quality {
inline bool contains(const Cell& c, Edge e)
{
    return std::find(c.v.begin(), c.v.end(), e[0]) != c.v.end() &&
           std::find(c.v.begin(), c.v.end(), e[1]) != c.v.end();
}
inline double shell_excess(const Mesh& m, const std::vector<Cell>& cc)
{
    long double e = 0;
    for (const auto& c : cc)
        if (c.type == 4 && c.region == 2)
            e += native::cell_volume(m.p, c) * std::pow(std::max(0., c.energy - 10), 2);
    return double(e);
}
inline bool patch_better(const Mesh& m, const std::vector<Cell>& a, const std::vector<Cell>& b)
{
    double old = shell_excess(m, a), now = shell_excess(m, b);
    if (old > 0) return now < old * (1 - 1e-8);
    long double x = 0, y = 0;
    for (const auto& c : a) x += native::cell_volume(m.p, c) * c.energy;
    for (const auto& c : b) y += native::cell_volume(m.p, c) * c.energy;
    return y < x * (1 - 1e-9L);
}
struct TetSide
{
    std::set<size_t> cells;
    std::vector<Cell> replacement;
};
inline TetSide
retile_tet_side(const Mesh& m, Edge edge, Edge ends, const std::set<size_t>& allowed_native)
{
    TetSide out;
    std::map<int, std::map<size_t, std::set<size_t>>> graphs;
    for (size_t ci : m.star[edge[0]])
        if (contains(m.cells[ci], edge)) {
            const auto& c = m.cells[ci];
            if (c.type != 4) {
                if (!allowed_native.count(ci))
                    throw std::runtime_error("swap_edge_touches_unrelated_native");
                continue;
            }
            out.cells.insert(ci);
            std::vector<size_t> link;
            for (size_t v : c.v)
                if (v != edge[0] && v != edge[1]) link.push_back(v);
            if (link.size() != 2) throw std::runtime_error("bad_tet_link");
            auto& g = graphs[c.region];
            if (!g[link[0]].insert(link[1]).second) throw std::runtime_error("duplicate_link");
            g[link[1]].insert(link[0]);
        }
    if (out.cells.empty()) throw std::runtime_error("missing_opposite_tet_side");
    for (auto& [region, graph] : graphs) {
        for (const auto& [v, neighbors] : graph)
            if (neighbors.size() != size_t(v == ends[0] || v == ends[1] ? 1 : 2))
                throw std::runtime_error("opposite_tet_link_not_path");
        std::vector<size_t> path{ends[0]};
        size_t previous = size_t(-1), v = ends[0];
        while (v != ends[1]) {
            auto it = graph.find(v);
            if (it == graph.end()) throw std::runtime_error("opposite_link_endpoint_missing");
            size_t next = size_t(-1);
            for (size_t w : it->second)
                if (w != previous) {
                    next = w;
                    break;
                }
            if (next == size_t(-1) || path.size() > graph.size())
                throw std::runtime_error("opposite_link_cycle");
            path.push_back(next);
            previous = v;
            v = next;
        }
        if (path.size() != graph.size() || path.size() > 32)
            throw std::runtime_error("opposite_link_disconnected_or_large");
        size_t n = path.size();
        if (n == 2) continue;
        std::vector<std::vector<double>> best(n, std::vector<double>(n, INFINITY));
        std::vector<std::vector<int>> cut(n, std::vector<int>(n, -1));
        auto pair = [&](size_t i, size_t k, size_t j) {
            std::array<Cell, 2> cc{
                Cell{4, region, {edge[0], path[i], path[k], path[j]}},
                Cell{4, region, {edge[1], path[i], path[j], path[k]}}};
            for (auto& c : cc) {
                if (native::determinant(m.p, c.v) < 0) std::swap(c.v[2], c.v[3]);
                if (!std::isfinite(native::quality(m.p, c)))
                    throw std::runtime_error("swap_nonpositive_tet");
            }
            return cc;
        };
        for (size_t i = 0; i + 1 < n; ++i) best[i][i + 1] = 0;
        for (size_t len = 2; len < n; ++len)
            for (size_t i = 0; i + len < n; ++i) {
                size_t j = i + len;
                for (size_t k = i + 1; k < j; ++k)
                    if (std::isfinite(best[i][k]) && std::isfinite(best[k][j])) try {
                            const auto cc = pair(i, k, j);
                            double q =
                                std::max({best[i][k], best[k][j], cc[0].energy, cc[1].energy});
                            if (q < best[i][j]) {
                                best[i][j] = q;
                                cut[i][j] = k;
                            }
                        } catch (const std::exception&) {
                        }
            }
        if (cut[0][n - 1] < 0) throw std::runtime_error("no_positive_opposite_tet_retiling");
        std::function<void(size_t, size_t)> emit = [&](size_t i, size_t j) {
            if (j <= i + 1) return;
            size_t k = cut[i][j];
            auto cc = pair(i, k, j);
            out.replacement.insert(out.replacement.end(), cc.begin(), cc.end());
            emit(i, k);
            emit(k, j);
        };
        emit(0, n - 1);
    }
    return out;
}
inline void no_existing_edge_outside(const Mesh& m, Edge e, const std::set<size_t>& patch)
{
    for (size_t ci : m.star[e[0]])
        if (contains(m.cells[ci], e) && !patch.count(ci))
            throw std::runtime_error("new_diagonal_already_exists");
}
inline void replace_patch(Mesh& m, const std::set<size_t>& patch, std::vector<Cell> replacement)
{
    for (size_t ci : patch)
        if (m.fixed(m.cells[ci])) throw std::runtime_error("patch_touches_reserved_rod_cell");
    std::vector<Cell> cells;
    cells.reserve(m.cells.size() - patch.size() + replacement.size());
    for (size_t i = 0; i < m.cells.size(); ++i)
        if (!patch.count(i)) cells.push_back(std::move(m.cells[i]));
    cells.insert(
        cells.end(),
        std::make_move_iterator(replacement.begin()),
        std::make_move_iterator(replacement.end()));
    m.cells = std::move(cells);
    m.rebuild();
}
inline Json flip_input_edge(
    Mesh& m,
    Rules& rules,
    Edge edge,
    bool enforce_quality = true,
    BoundarySurface* surface = nullptr)
{
    std::vector<std::pair<Key, InputFace>> old_faces;
    std::set<size_t> other;
    for (const auto& [key, f] : rules.kept) {
        if (std::find(f.v.begin(), f.v.end(), edge[0]) != f.v.end() &&
            std::find(f.v.begin(), f.v.end(), edge[1]) != f.v.end()) {
            old_faces.push_back({key, f});
            for (size_t v : f.v)
                if (v != edge[0] && v != edge[1]) other.insert(v);
        }
    }
    if (old_faces.size() != 2 || other.size() != 2)
        throw std::runtime_error("input_edge_not_two_triangles");
    const auto proto = old_faces[0].second;
    if (old_faces[1].second.patch != proto.patch || old_faces[1].second.kind != proto.kind)
        throw std::runtime_error("input_feature_or_case_boundary");
    if (proto.kind == 3) throw std::runtime_error("native_only_input_not_selected");
    Edge newedge{*other.begin(), *other.rbegin()};
    for (const auto& [key, f] : rules.kept)
        if (std::find(f.v.begin(), f.v.end(), newedge[0]) != f.v.end() &&
            std::find(f.v.begin(), f.v.end(), newedge[1]) != f.v.end())
            throw std::runtime_error("input_new_diagonal_already_present");
    std::set<size_t> prisms;
    for (size_t ci : m.star[edge[0]])
        if (contains(m.cells[ci], edge) && m.cells[ci].type != 4) {
            if (m.cells[ci].type != 6) throw std::runtime_error("input_swap_opposite_pyramid");
            prisms.insert(ci);
        }
    std::vector<Cell> replacement;
    std::set<size_t> patch = prisms;
    const auto input_side = retile_tet_side(m, edge, newedge, prisms);
    patch.insert(input_side.cells.begin(), input_side.cells.end());
    replacement = input_side.replacement;
    Edge offset_edge{}, offset_new{};
    if (proto.kind == 2) {
        if (prisms.size() != 2) throw std::runtime_error("opposite_prism_pair_not_two");
        std::map<size_t, size_t> matching;
        for (size_t ci : prisms) {
            const auto& c = m.cells[ci];
            for (int j = 0; j < 3; ++j) {
                size_t a = c.v[j], b = c.v[j + 3];
                if (!m.input[a] || m.input[b]) throw std::runtime_error("prism_cap_input_order");
                if (m.correspondence[b] != int64_t(a))
                    throw std::runtime_error("prism_cap_correspondence");
                auto [it, inserted] = matching.emplace(a, b);
                if (!inserted && it->second != b)
                    throw std::runtime_error("prisms_on_different_input_sides");
            }
        }
        for (size_t v : {edge[0], edge[1], newedge[0], newedge[1]})
            if (!matching.count(v)) throw std::runtime_error("prism_missing_paired_cap");
        offset_edge = native::edge(matching[edge[0]], matching[edge[1]]);
        offset_new = native::edge(matching[newedge[0]], matching[newedge[1]]);
        const auto offset_side = retile_tet_side(m, offset_edge, offset_new, prisms);
        patch.insert(offset_side.cells.begin(), offset_side.cells.end());
        replacement.insert(
            replacement.end(),
            offset_side.replacement.begin(),
            offset_side.replacement.end());
        for (size_t endpoint : edge) {
            std::vector<size_t> v{
                newedge[0],
                newedge[1],
                endpoint,
                matching[newedge[0]],
                matching[newedge[1]],
                matching[endpoint]};
            Cell c{6, 2, v};
            if (!std::isfinite(native::quality(m.p, c))) {
                std::swap(c.v[1], c.v[2]);
                std::swap(c.v[4], c.v[5]);
                if (!std::isfinite(native::quality(m.p, c)))
                    throw std::runtime_error("flipped_prism_jacobian");
            }
            replacement.push_back(c);
        }
    } else if (!prisms.empty())
        throw std::runtime_error("single_sided_input_swap_touches_native");
    no_existing_edge_outside(m, newedge, patch);
    if (proto.kind == 2) no_existing_edge_outside(m, offset_new, patch);
    std::vector<Cell> old;
    for (size_t ci : patch) old.push_back(m.cells[ci]);
    if (cell_boundary(old) != cell_boundary(replacement))
        throw std::runtime_error("coupled_swap_cavity_boundary");
    long double a = 0, b = 0;
    for (const auto& c : old) a += native::cell_volume(m.p, c);
    for (const auto& c : replacement) b += native::cell_volume(m.p, c);
    if (std::abs(a - b) > 1e-10L * std::max(1e-10L, std::abs(a)))
        throw std::runtime_error("coupled_swap_volume");
    std::array<InputFace, 2> ff{proto, proto};
    for (int i = 0; i < 2; ++i) {
        ff[i].v = {newedge[0], newedge[1], edge[i]};
        rules.orient(m, ff[i]);
        if (!(rules.area(m, ff[i]) > 0)) throw std::runtime_error("input_surface_flip_nonconvex");
    }
    const double old_area = rules.area(m, old_faces[0].second) + rules.area(m, old_faces[1].second),
                 new_area = rules.area(m, ff[0]) + rules.area(m, ff[1]);
    if (std::abs(old_area - new_area) > 1e-10 * old_area)
        throw std::runtime_error("input_surface_flip_coverage");
    std::vector<Tri> old_surface{old_faces[0].second.v, old_faces[1].second.v},
        next_surface{ff[0].v, ff[1].v};
    if (proto.kind == 2) {
        old_surface.push_back({offset_edge[0], offset_edge[1], offset_new[0]});
        old_surface.push_back({offset_edge[1], offset_edge[0], offset_new[1]});
        next_surface.push_back({offset_new[0], offset_new[1], offset_edge[0]});
        next_surface.push_back({offset_new[1], offset_new[0], offset_edge[1]});
    }
    const auto decision = surface_decision(m, surface, edge, newedge, old_surface, next_surface);
    if (enforce_quality && !decision.accepted)
        throw std::runtime_error("boundary_triangle_shape_and_degree_not_improved");
    for (const auto& f : old_faces) rules.kept.erase(f.first);
    for (const auto& f : ff) rules.kept.emplace(native::facekey({f.v[0], f.v[1], f.v[2]}), f);
    const double old_excess = shell_excess(m, old), new_excess = shell_excess(m, replacement);
    const size_t new_count = replacement.size();
    if (surface) surface->replace(old, replacement);
    replace_patch(m, patch, std::move(replacement));
    return {
        {"edge", edge},
        {"new_input_edge", newedge},
        {"case", proto.kind},
        {"surface_decision", decision.json()},
        {"opposite_prisms_updated", prisms.size()},
        {"old_cells", old.size()},
        {"new_cells", new_count},
        {"shell_excess_before", old_excess},
        {"shell_excess_after", new_excess},
        {"tet_energy_increased", new_excess > old_excess * (1 + 1e-9)},
        {"complete_cavity_boundary_preserved", true}};
}
inline Json surface_splits(
    Mesh& m,
    Rules& rules,
    double length,
    size_t budget,
    Deadline& deadline,
    const std::vector<uint8_t>* adaptive_region = nullptr,
    const std::function<void(size_t, Edge)>& midpoint_callback = {})
{
    std::map<Edge, double> priority;
    for (const auto& c : m.cells) {
        if (!adaptive_region && c.region != 2 && c.energy <= 20) continue;
        for (auto e : native::cell_edges(c)) {
            if (adaptive_region && (!adaptive_region->at(e[0]) || !adaptive_region->at(e[1])))
                continue;
            const double s = .5 * (m.sizing[e[0]] + m.sizing[e[1]]);
            if ((m.p[e[0]] - m.p[e[1]]).norm() > 1.4 * length * s) {
                const double q = c.energy;
                priority[e] = std::max(priority[e], q);
            }
        }
    }
    std::vector<Edge> edges;
    for (const auto& [e, q] : priority) edges.push_back(e);
    std::sort(edges.begin(), edges.end(), [&](Edge a, Edge b) {
        if (priority[a] != priority[b]) return priority[a] > priority[b];
        return (m.p[a[0]] - m.p[a[1]]).squaredNorm() > (m.p[b[0]] - m.p[b[1]]).squaredNorm();
    });
    native::ConformingRefiner refiner(m);
    size_t tried = 0, accepted = 0, input_splits = 0, native_prisms = 0;
    Json rejects = Json::object();
    for (auto e : edges) {
        if (accepted >= budget || tried >= budget * 10 || deadline.expired()) break;
        ++tried;
        const auto plan = refiner.closure(e, true, 2000, true, false);
        if (plan.reason != "ok") {
            rejects[plan.reason] = rejects.value(plan.reason, size_t(0)) + 1;
            continue;
        }
        bool boundary = false;
        for (const auto& [k, f] : rules.kept)
            for (int j = 0; j < 3; ++j)
                if (plan.marks.count(native::edge(f.v[j], f.v[(j + 1) % 3]))) boundary = true;
        const size_t n = m.p.size();
        if (!refiner.split(e, true, 2000, true, false, true)) {
            continue;
        }
        std::map<Edge, size_t> mid;
        for (size_t v = n; v < m.p.size(); ++v) {
            const auto parent = refiner.point_parents[v];
            mid[parent] = v;
            if (!m.input[v] && !std::isfinite(m.thickness[v]) &&
                std::isfinite(m.thickness[parent[0]]) && std::isfinite(m.thickness[parent[1]])) {
                m.base[v] = (m.base[parent[0]] + m.base[parent[1]]) / 2;
                m.initial[v] = (m.initial[parent[0]] + m.initial[parent[1]]) / 2;
                m.thickness[v] = (m.initial[v] - m.base[v]).norm();
                m.correspondence[v] = -3;
            }
            if (midpoint_callback) midpoint_callback(v, parent);
        }
        rules.split_labels(m, mid);
        ++accepted;
        input_splits += boundary;
        const auto& event = refiner.events.back();
        if (event["old_types"].contains("6"))
            native_prisms += event["old_types"]["6"].get<size_t>();
    }
    auto report = refiner.validate();
    report["adaptive_field_driven"] = adaptive_region != nullptr;
    report["candidate_edges"] = edges.size();
    report["attempted_seeds"] = tried;
    report["accepted_seeds"] = accepted;
    report["protected_input_split_transactions"] = input_splits;
    report["opposite_prisms_subdivided"] = native_prisms;
    report["closure_rejections"] = rejects;
    report["events"] = refiner.events;
    refiner.compact();
    report["new_tet_tet_input_faces_released"] = rules.update_cases(m);
    rules.refresh_roles(m);
    return report;
}
} // namespace wmtk::components::prismatic_mesh::quality
