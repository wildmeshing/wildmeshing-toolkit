#pragma once
#include "mixed_optimizer.hpp"
#include "surface_ops.hpp"
namespace wmtk::components::prismatic_mesh::quality {
inline TetSide optional_tet_side(const Mesh& m, Edge e, Edge other, const std::set<size_t>& allowed)
{
    bool found = false;
    for (size_t ci : m.star[e[0]])
        if (contains(m.cells[ci], e) && m.cells[ci].type == 4) found = true;
    if (found) return retile_tet_side(m, e, other, allowed);
    for (size_t ci : m.star[e[0]])
        if (contains(m.cells[ci], e) && !allowed.count(ci))
            throw std::runtime_error("cap_touches_unsupported_native");
    return {};
}
inline Json flip_prism_region(Mesh& m, Rules& rules, Edge edge)
{
    std::vector<std::pair<Key, InputFace>> old_faces;
    std::set<size_t> ends;
    for (const auto& [k, f] : rules.kept)
        if (std::find(f.v.begin(), f.v.end(), edge[0]) != f.v.end() &&
            std::find(f.v.begin(), f.v.end(), edge[1]) != f.v.end()) {
            old_faces.push_back({k, f});
            for (size_t v : f.v)
                if (v != edge[0] && v != edge[1]) ends.insert(v);
        }
    if (old_faces.size() != 2 || ends.size() != 2)
        throw std::runtime_error("prism_input_edge_not_manifold");
    const auto proto = old_faces[0].second;
    if (old_faces[1].second.patch != proto.patch)
        throw std::runtime_error("prism_input_feature_edge");
    Edge other{*ends.begin(), *ends.rbegin()};
    std::set<size_t> prisms;
    std::map<Edge, std::set<size_t>> layers;
    for (size_t ci : m.star[edge[0]])
        if (contains(m.cells[ci], edge) && m.cells[ci].type != 4) {
            const auto& c = m.cells[ci];
            if (c.type != 6) throw std::runtime_error("prism_flip_touches_pyramid");
            std::map<size_t, size_t> cap;
            for (int j = 0; j < 3; ++j) {
                if (!m.input[c.v[j]] || m.input[c.v[j + 3]] ||
                    m.correspondence[c.v[j + 3]] != int64_t(c.v[j]))
                    throw std::runtime_error("prism_column_correspondence");
                cap[c.v[j]] = c.v[j + 3];
            }
            if (!cap.count(edge[0]) || !cap.count(edge[1]))
                throw std::runtime_error("not_a_prism_cap_edge");
            prisms.insert(ci);
            layers[native::edge(cap[edge[0]], cap[edge[1]])].insert(ci);
        }
    if (prisms.empty()) return flip_input_edge(m, rules, edge, true);
    std::set<size_t> patch = prisms;
    std::vector<Cell> next;
    auto input = optional_tet_side(m, edge, other, prisms);
    patch.insert(input.cells.begin(), input.cells.end());
    next = input.replacement;
    std::vector<Tri> before{old_faces[0].second.v, old_faces[1].second.v},
        after{{other[0], other[1], edge[0]}, {other[1], other[0], edge[1]}};
    for (const auto& [offset_edge, group] : layers) {
        if (group.size() != 2) throw std::runtime_error("prism_layer_not_a_pair");
        std::map<size_t, size_t> cap;
        for (size_t ci : group) {
            const auto& c = m.cells[ci];
            for (int j = 0; j < 3; ++j) {
                auto [it, ok] = cap.emplace(c.v[j], c.v[j + 3]);
                if (!ok && it->second != c.v[j + 3])
                    throw std::runtime_error("prism_layer_column_conflict");
            }
        }
        for (size_t v : {edge[0], edge[1], other[0], other[1]})
            if (!cap.count(v)) throw std::runtime_error("prism_quad_missing_column");
        const Edge new_offset = native::edge(cap[other[0]], cap[other[1]]);
        auto side = optional_tet_side(m, offset_edge, new_offset, group);
        patch.insert(side.cells.begin(), side.cells.end());
        next.insert(next.end(), side.replacement.begin(), side.replacement.end());
        no_existing_edge_outside(m, new_offset, patch);
        before.push_back({cap[edge[0]], cap[edge[1]], cap[other[0]]});
        before.push_back({cap[edge[1]], cap[edge[0]], cap[other[1]]});
        after.push_back({cap[other[0]], cap[other[1]], cap[edge[0]]});
        after.push_back({cap[other[1]], cap[other[0]], cap[edge[1]]});
        for (size_t endpoint : edge) {
            Cell c{
                6,
                2,
                {other[0], other[1], endpoint, cap[other[0]], cap[other[1]], cap[endpoint]}};
            if (!std::isfinite(native::quality(m.p, c))) {
                std::swap(c.v[0], c.v[1]);
                std::swap(c.v[3], c.v[4]);
                if (!std::isfinite(native::quality(m.p, c)))
                    throw std::runtime_error("prism_region_flip_inverted");
            }
            next.push_back(std::move(c));
        }
    }
    const auto decision = surface_decision(m, nullptr, edge, other, before, after);
    if (!decision.accepted) throw std::runtime_error("prism_cap_shape_not_improved");
    no_existing_edge_outside(m, other, patch);
    std::vector<Cell> old;
    for (size_t ci : patch) old.push_back(m.cells[ci]);
    if (cell_boundary(old) != cell_boundary(next))
        throw std::runtime_error("prism_region_cavity_boundary_changed");
    auto faces = faces_of(next);
    for (const auto& [k, f] : faces_of(old))
        if (rules.kept.count(k) && k != old_faces[0].first && k != old_faces[1].first &&
            !faces.count(k))
            throw std::runtime_error("prism_region_flip_input_face_lost");
    std::array<InputFace, 2> ff{proto, proto};
    for (int j = 0; j < 2; ++j) {
        ff[j].v = {other[0], other[1], edge[j]};
        rules.orient(m, ff[j]);
        if (!(rules.area(m, ff[j]) > 0))
            throw std::runtime_error("prism_region_nonconvex_input_quad");
    }
    const double area0 = rules.area(m, old_faces[0].second) + rules.area(m, old_faces[1].second),
                 area1 = rules.area(m, ff[0]) + rules.area(m, ff[1]);
    if (std::abs(area0 - area1) > 1e-10 * area0)
        throw std::runtime_error("prism_region_input_coverage");
    for (const auto& f : old_faces) rules.kept.erase(f.first);
    for (const auto& f : ff) rules.kept[native::facekey({f.v[0], f.v[1], f.v[2]})] = f;
    const size_t n = next.size();
    replace_patch(m, patch, std::move(next));
    return {
        {"edge", edge},
        {"new_edge", other},
        {"prism_layers", layers.size()},
        {"prisms_updated", prisms.size()},
        {"old_cells", old.size()},
        {"new_cells", n},
        {"surface_decision", decision.json()}};
}
inline Json prism_region_flips(Mesh& m, Rules& rules, size_t attempts, Deadline& deadline)
{
    std::map<Edge, std::vector<InputFace>> adjacent;
    for (const auto& [k, f] : rules.kept)
        for (int j = 0; j < 3; ++j) adjacent[native::edge(f.v[j], f.v[(j + 1) % 3])].push_back(f);
    std::vector<std::pair<double, Edge>> candidate;
    Json rejected = Json::object(), ops = Json::array();
    for (const auto& [e, faces] : adjacent)
        if (faces.size() == 2 && faces[0].patch == faces[1].patch) {
            std::set<size_t> ends;
            for (const auto& f : faces)
                for (size_t v : f.v)
                    if (v != e[0] && v != e[1]) ends.insert(v);
            if (ends.size() != 2) continue;
            Edge next{*ends.begin(), *ends.rbegin()};
            auto d = surface_decision(
                m,
                nullptr,
                e,
                next,
                {faces[0].v, faces[1].v},
                {{next[0], next[1], e[0]}, {next[1], next[0], e[1]}});
            if (d.accepted) candidate.push_back({d.after_min - d.before_min, e});
        }
    std::sort(candidate.begin(), candidate.end(), [](const auto& a, const auto& b) {
        return a.first != b.first ? a.first > b.first : a.second < b.second;
    });
    size_t tried = 0;
    for (const auto& [q, e] : candidate) {
        if (tried >= attempts || deadline.expired()) break;
        ++tried;
        try {
            ops.push_back(flip_prism_region(m, rules, e));
        } catch (const std::exception& error) {
            rejected[error.what()] = rejected.value(error.what(), size_t(0)) + 1;
        }
    }
    return {
        {"attempted", tried},
        {"accepted", ops.size()},
        {"operations", ops},
        {"rejections", rejected},
        {"native_only_prism_regions_included", true}};
}
inline std::set<std::vector<size_t>> surface_link(
    const Rules& rules,
    const std::vector<size_t>& simplex)
{
    std::set<std::vector<size_t>> out;
    for (const auto& [k, f] : rules.kept) {
        bool has = true;
        for (size_t v : simplex) has &= std::find(f.v.begin(), f.v.end(), v) != f.v.end();
        if (!has) continue;
        std::vector<size_t> rest;
        for (size_t v : f.v)
            if (std::find(simplex.begin(), simplex.end(), v) == simplex.end()) rest.push_back(v);
        for (size_t mask = 0; mask < (size_t(1) << rest.size()); ++mask) {
            std::vector<size_t> a;
            for (size_t j = 0; j < rest.size(); ++j)
                if (mask & (size_t(1) << j)) a.push_back(rest[j]);
            std::sort(a.begin(), a.end());
            out.insert(a);
        }
    }
    return out;
}
inline std::set<std::vector<size_t>>
cap_link(const Mesh& m, const std::vector<size_t>& simplex, int region)
{
    std::set<std::vector<size_t>> out;
    for (size_t ci : m.star[simplex[0]]) {
        const auto& c = m.cells[ci];
        if (c.type != 4 || (region && c.region != region)) continue;
        bool has = true;
        for (size_t v : simplex) has &= std::find(c.v.begin(), c.v.end(), v) != c.v.end();
        if (!has) continue;
        std::vector<size_t> rest;
        for (size_t v : c.v)
            if (std::find(simplex.begin(), simplex.end(), v) == simplex.end()) rest.push_back(v);
        for (size_t mask = 0; mask < (size_t(1) << rest.size()); ++mask) {
            std::vector<size_t> a;
            for (size_t j = 0; j < rest.size(); ++j)
                if (mask & (size_t(1) << j)) a.push_back(rest[j]);
            std::sort(a.begin(), a.end());
            out.insert(a);
        }
    }
    return out;
}
inline void require_link(
    const std::set<std::vector<size_t>>& a,
    const std::set<std::vector<size_t>>& b,
    const std::set<std::vector<size_t>>& e)
{
    std::set<std::vector<size_t>> both;
    std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::inserter(both, both.end()));
    if (both != e) throw std::runtime_error("column_collapse_link_condition");
}
inline Json collapse_prism_column(Mesh& m, Rules& rules, size_t a, size_t b)
{
    if (!m.input[a] || !m.input[b] || m.outer[a] || m.outer[b])
        throw std::runtime_error("column_collapse_not_input_edge");
    InputFreedom freedom(m, rules);
    const V delta = m.p[b] - m.p[a];
    if ((delta - freedom.tangent[a] * delta).norm() > 1e-9)
        throw std::runtime_error("column_collapse_input_feature");
    require_link(surface_link(rules, {a}), surface_link(rules, {b}), surface_link(rules, {a, b}));
    std::map<size_t, size_t> mapping{{a, b}};
    size_t native_count = 0;
    for (size_t ci : m.star[a])
        if (contains(m.cells[ci], native::edge(a, b)) && m.cells[ci].type == 6) {
            const auto& c = m.cells[ci];
            std::map<size_t, size_t> cap;
            for (int j = 0; j < 3; ++j) cap[c.v[j]] = c.v[j + 3];
            if (!cap.count(a) || !cap.count(b)) throw std::runtime_error("column_not_cap_edge");
            auto [it, ok] = mapping.emplace(cap[a], cap[b]);
            if (!ok && it->second != cap[b])
                throw std::runtime_error("column_collapse_layer_mismatch");
            ++native_count;
        }
    if (!native_count) throw std::runtime_error("column_edge_has_no_prism");
    for (size_t child : freedom.children[a])
        if (!mapping.count(child) && !m.star[child].empty())
            throw std::runtime_error("column_collapse_unmatched_offset_child");
    std::set<size_t> patch;
    for (auto [u, v] : mapping) {
        if (u == v || m.input[u] != m.input[v] || m.outer[u] || mapping.count(v))
            throw std::runtime_error("column_collapse_bad_pair");
        for (int region : {0, 2, 3})
            require_link(
                cap_link(m, {u}, region),
                cap_link(m, {v}, region),
                cap_link(m, {u, v}, region));
        patch.insert(m.star[u].begin(), m.star[u].end());
    }
    std::vector<Cell> old, next;
    std::set<std::pair<int, std::vector<size_t>>> unique;
    for (size_t ci : patch) {
        const auto& source = m.cells[ci];
        old.push_back(source);
        if (source.type == 7) throw std::runtime_error("column_collapse_pyramid_junction");
        auto c = source;
        for (size_t& v : c.v)
            if (mapping.count(v)) v = mapping.at(v);
        const size_t n = std::set<size_t>(c.v.begin(), c.v.end()).size();
        if (n < c.v.size()) {
            if (c.type == 4 || (c.type == 6 && n == 4)) continue;
            throw std::runtime_error("column_collapse_unsupported_degeneracy");
        }
        if (!std::isfinite(native::quality(m.p, c)))
            throw std::runtime_error("column_collapse_inverted_cell");
        auto key = c.v;
        std::sort(key.begin(), key.end());
        if (!unique.insert({c.region, key}).second)
            throw std::runtime_error("column_collapse_duplicate_cell");
        for (size_t outside : m.star[c.v[0]])
            if (!patch.count(outside)) {
                auto other = m.cells[outside].v;
                std::sort(other.begin(), other.end());
                if (other == key) throw std::runtime_error("column_collapse_existing_cell");
            }
        next.push_back(std::move(c));
    }
    if (next.empty() || cell_boundary(old) != cell_boundary(next))
        throw std::runtime_error("column_collapse_cavity_boundary");
    auto kept = rules.kept;
    std::map<Key, InputFace> updated;
    std::vector<double> coverage(rules.patch_area.size());
    for (const auto& [key, face] : kept) {
        auto f = face;
        for (size_t& v : f.v)
            if (mapping.count(v)) v = mapping.at(v);
        if (std::set<size_t>(f.v.begin(), f.v.end()).size() < 3) continue;
        const auto k = native::facekey({f.v[0], f.v[1], f.v[2]});
        if (!(rules.area(m, f) > 0) || !updated.emplace(k, f).second)
            throw std::runtime_error("column_collapse_surface_fold");
        for (size_t v : f.v)
            if (std::abs(rules.patch_normal[f.patch].dot(m.p[v] - rules.patch_origin[f.patch])) >
                1e-9)
                throw std::runtime_error("column_collapse_input_geometry");
        coverage[f.patch] += rules.area(m, f);
    }
    for (size_t i = 0; i < coverage.size(); ++i)
        if (std::abs(coverage[i] - rules.patch_area[i]) > 1e-9 * std::max(1., rules.patch_area[i]))
            throw std::runtime_error("column_collapse_input_area");
    const size_t before = old.size(), after = next.size();
    rules.kept = std::move(updated);
    replace_patch(m, patch, std::move(next));
    return {
        {"input_removed", a},
        {"input_survivor", b},
        {"paired_column_vertices", mapping.size()},
        {"old_cells", before},
        {"new_cells", after},
        {"surface_and_material_tet_cap_links_checked", true}};
}
inline Json
prism_region_collapses(Mesh& m, Rules& rules, double length, size_t attempts, Deadline& deadline)
{
    std::set<Edge> seen;
    std::vector<std::pair<double, Edge>> candidates;
    for (const auto& [k, f] : rules.kept)
        for (int j = 0; j < 3; ++j) {
            const auto e = native::edge(f.v[j], f.v[(j + 1) % 3]);
            if (!seen.insert(e).second) continue;
            const double l = (m.p[e[0]] - m.p[e[1]]).norm(),
                         target = length * .5 * (m.sizing[e[0]] + m.sizing[e[1]]);
            if (l < .8 * target) candidates.push_back({l, e});
        }
    std::sort(candidates.begin(), candidates.end());
    Json ops = Json::array(), reject = Json::object();
    size_t n = 0;
    for (auto [l, e] : candidates) {
        if (n >= attempts || deadline.expired()) break;
        for (int direction = 0; direction < 2 && n < attempts; ++direction) {
            ++n;
            try {
                ops.push_back(collapse_prism_column(m, rules, e[direction], e[1 - direction]));
                break;
            } catch (const std::exception& error) {
                reject[error.what()] = reject.value(error.what(), size_t(0)) + 1;
            }
        }
    }
    rules.refresh_roles(m);
    return {
        {"attempted", n},
        {"accepted", ops.size()},
        {"operations", ops},
        {"rejections", reject}};
}
} // namespace wmtk::components::prismatic_mesh::quality
