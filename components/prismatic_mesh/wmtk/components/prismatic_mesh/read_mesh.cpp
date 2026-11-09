#include "read_mesh.hpp"

#include <stdexcept>
#include <unordered_map>

namespace wmtk::components::prismatic_mesh {
void detail::resolve_layered_correspondence(PrismaticMeshInput& input)
{
    const size_t n = input.vertices.rows();
    std::unordered_map<int64_t, size_t> rows;
    for (size_t v = 0; v < n; ++v)
        if (!rows.emplace(input.source_vertex_ids[v], v).second)
            throw std::runtime_error("prismatic_mesh: duplicate vid");
    const auto raw = input.corr_input_vid;
    std::vector<int> state(n, 0), depth(n, 0);
    std::vector<int64_t> roots(n, -1);
    for (size_t v : input.input_vertices) {
        roots[v] = v;
        state[v] = 2;
    }
    std::set<size_t> shared, descendants, core_offsets;
    // Source curve/sheet vertices retain their input role even if no marching
    // edge in a later layer happens to use them as correspondence targets.
    for (const auto& f : input.source_faces)
        for (size_t v : f)
            if (input.vertex_tags[v] == 2) shared.insert(v);
    for (const auto& e : input.source_edges)
        for (size_t v : e)
            if (input.vertex_tags[v] == 2) shared.insert(v);
    for (size_t v : input.offset_vertices) {
        const auto target = rows.find(raw[v]);
        if (target == rows.end())
            throw std::runtime_error(
                "prismatic_mesh: offset vertex has missing or unknown input correspondence");
        if (input.vertex_tags[target->second] == 2) shared.insert(target->second);
        std::vector<size_t> path;
        size_t current = v;
        while (state[current] == 0) {
            if (input.vertex_tags[current] != 2)
                throw std::runtime_error(
                    "prismatic_mesh: correspondence target is not an input vertex");
            state[current] = 1;
            path.push_back(current);
            const auto next = rows.find(raw[current]);
            if (next == rows.end())
                throw std::runtime_error(
                    "prismatic_mesh: offset vertex has missing or unknown input correspondence");
            current = next->second;
        }
        if (state[current] == 1)
            throw std::runtime_error("prismatic_mesh: cyclic offset correspondence");
        for (auto it = path.rbegin(); it != path.rend(); ++it) {
            roots[*it] = roots[current];
            depth[*it] = depth[current] + 1;
            state[*it] = 2;
            current = *it;
        }
    }
    input.input_to_offset_vertices.assign(n, {});
    input.corr_input_vertex.assign(n, -1);
    nlohmann::json links = nlohmann::json::array();
    for (size_t v : input.offset_vertices) {
        const size_t root = roots[v];
        input.corr_input_vertex[v] = root;
        input.corr_input_vid[v] = input.source_vertex_ids[root];
        input.input_to_offset_vertices[root].push_back(v);
        if (depth[v] <= 1) continue;
        descendants.insert(v);
        links.push_back(
            {{"source_vid", input.source_vertex_ids[v]},
             {"original_target_vid", raw[v]},
             {"root_target_vid", input.source_vertex_ids[root]},
             {"depth", depth[v]}});
    }
    for (size_t tid = 0; tid < input.input_cells.size(); ++tid)
        if (input.input_cells[tid] == 1)
            for (int j = 0; j < 4; ++j) {
                const size_t v = input.tetrahedra(tid, j);
                if (input.vertex_tags[v] == 2) core_offsets.insert(v);
            }
    input.layered_junction_vertices.insert(shared.begin(), shared.end());
    input.layered_junction_vertices.insert(descendants.begin(), descendants.end());
    input.layered_junction_vertices.insert(core_offsets.begin(), core_offsets.end());
    if (!input.layered_junction_vertices.empty()) {
        const auto ids = [&](const auto& vertices) {
            std::vector<int64_t> result;
            for (size_t v : vertices) result.push_back(input.source_vertex_ids[v]);
            return result;
        };
        input.layered_correspondence_report = {
            {"policy",
             "retain original layer links; root correspondence only for protected junction fibers"},
            {"shared_role_source_vids", ids(shared)},
            {"core_offset_source_vids", ids(core_offsets)},
            {"protected_junction_source_vids", ids(input.layered_junction_vertices)},
            {"normalized_links", links}};
    }
}

PrismaticMeshInput load_prismatic_mesh(const std::filesystem::path& path)
{
    if (path.extension() == ".vtu") return detail::load_prismatic_vtu(path);
    if (path.extension() == ".msh") return detail::load_prismatic_msh(path);
    throw std::runtime_error("prismatic_mesh: input must be a .vtu or .msh file");
}
} // namespace wmtk::components::prismatic_mesh
