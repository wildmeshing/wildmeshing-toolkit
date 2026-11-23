#pragma once
#include "common.hpp"
namespace wmtk::components::prismatic_mesh::quality {
inline Json partition_regions(const Mesh& m, const Rules& rules, const std::vector<uint8_t>& offset)
{
    auto ff = faces_of(m.cells);
    DSU dsu(m.cells.size());
    auto name = [&](size_t ci) {
        const auto& c = m.cells[ci];
        return std::to_string(c.region) + (c.type == 4 ? "_tet" : "_native");
    };
    std::map<std::string, size_t> interfaces, counts;
    std::map<std::string, std::set<size_t>> components;
    size_t no_reference_face = 0;
    for (const auto& [k, f] : ff)
        if (f.b != size_t(-1)) {
            if (name(f.a) == name(f.b))
                dsu.join(f.a, f.b);
            else {
                auto a = name(f.a), b = name(f.b);
                if (a > b) std::swap(a, b);
                ++interfaces[a + "|" + b];
            }
        }
    for (size_t i = 0; i < m.cells.size(); ++i) {
        ++counts[name(i)];
        components[name(i)].insert(dsu.get(i));
        if (m.cells[i].type == 4 && m.cells[i].region == 2) {
            bool reference = false;
            for (const auto& local : native::faces.at(4)) {
                bool input = true, on_offset = true;
                for (int j : local) {
                    size_t v = m.cells[i].v[j];
                    input &= m.input[v] ||
                             (v < rules.originally_input.size() && rules.originally_input[v]);
                    on_offset &= (v < offset.size() && offset[v]) ||
                                 (v >= offset.size() && !m.input[v] &&
                                  std::isfinite(m.thickness[v]) && m.thickness[v] > 0);
                }
                reference |= input || on_offset;
            }
            no_reference_face += !reference;
        }
    }
    Json parts = Json::object();
    for (const auto& [n, c] : counts)
        parts[n] = {{"cells", c}, {"face_connected_components", components[n].size()}};
    return {
        {"parts", parts},
        {"shared_interfaces", interfaces},
        {"shell_tets_without_input_or_offset_triangle", no_reference_face},
        {"tau2X_policy",
         "All actual tets are component domains; no source-tet boundary or target-position pins"}};
}
inline Json initialize_region_sizing(Mesh& m, double average, double factor, double thickness)
{
    const double target = std::max(thickness, average * factor);
    if (!(target > 0) || !(thickness > 0)) throw std::runtime_error("invalid_region_sizing");
    m.sizing.assign(m.p.size(), 1);
    return {
        {"mode", "offset_unique_edge_average"},
        {"offset_average", average},
        {"factor", factor},
        {"initial_target_length", target},
        {"minimum_length", thickness},
        {"minimum_scalar", thickness / target},
        {"target_clamped_to_thickness", average * factor < thickness}};
}
inline Json
refine_mixed_sizing(Mesh& m, double target_quality, double base, double floor, int rings)
{
    std::vector<uint8_t> selected(m.p.size(), false);
    size_t bad = 0;
    for (const auto& c : m.cells)
        if (c.energy > target_quality) {
            ++bad;
            for (size_t v : c.v) selected[v] = true;
        }
    for (int ring = 0; ring < rings; ++ring) {
        auto next = selected;
        for (const auto& c : m.cells) {
            bool near = false;
            for (size_t v : c.v) near |= selected[v];
            if (near)
                for (size_t v : c.v) next[v] = true;
        }
        selected = std::move(next);
    }
    std::vector<double> old = m.sizing;
    std::queue<size_t> pending;
    size_t refined = 0, at_floor = 0;
    const double minimum = floor / base;
    for (size_t v = 0; v < m.p.size(); ++v)
        if (selected[v]) {
            m.sizing[v] = std::max(minimum, m.sizing[v] * .5);
            if (m.sizing[v] < old[v] * (1 - 1e-12)) {
                ++refined;
                pending.push(v);
            } else
                ++at_floor;
        }
    std::vector<std::vector<size_t>> graph(m.p.size());
    for (const auto& c : m.cells)
        for (auto e : native::cell_edges(c)) {
            graph[e[0]].push_back(e[1]);
            graph[e[1]].push_back(e[0]);
        }
    for (size_t v = 0; v < m.p.size(); ++v)
        if (m.correspondence[v] >= 0) {
            const size_t u = m.correspondence[v];
            graph[v].push_back(u);
            graph[u].push_back(v);
        }
    size_t graded = 0;
    while (!pending.empty()) {
        const size_t v = pending.front();
        pending.pop();
        for (size_t w : graph[v]) {
            const bool column =
                m.correspondence[w] == int64_t(v) || m.correspondence[v] == int64_t(w);
            const double bound = std::max(minimum, m.sizing[v] * (column ? 1. : 1.4));
            if (m.sizing[w] > bound * (1 + 1e-12)) {
                m.sizing[w] = bound;
                pending.push(w);
                ++graded;
            }
        }
    }
    return {
        {"bad_mixed_cells", bad},
        {"directly_refined_vertices", refined},
        {"selected_vertices_already_at_floor", at_floor},
        {"gradation_updates", graded},
        {"minimum_scalar", *std::min_element(m.sizing.begin(), m.sizing.end())},
        {"minimum_length", base * *std::min_element(m.sizing.begin(), m.sizing.end())},
        {"floor", floor},
        {"threshold", target_quality}};
}
} // namespace wmtk::components::prismatic_mesh::quality
