#include "prismatic_mesh.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
using Edge = std::array<size_t, 2>;

Edge edge_key(size_t a, size_t b)
{
    return a < b ? Edge{a, b} : Edge{b, a};
}

bool is_volume_edge(const PrismaticMeshInput& input, const Edge& edge)
{
    const auto& a = input.mesh->get_one_ring_tids_for_vertex(edge[0]);
    const auto& b = input.mesh->get_one_ring_tids_for_vertex(edge[1]);
    for (size_t tid : a.size() <= b.size() ? a : b) {
        if (input.input_cells.at(tid) != 1) continue;
        const auto tet = input.mesh->oriented_tet_vids(tid);
        if (std::find(tet.begin(), tet.end(), edge[0]) != tet.end() &&
            std::find(tet.begin(), tet.end(), edge[1]) != tet.end())
            return true;
    }
    return false;
}
} // namespace

OpenBoundaryStatistics handle_open_boundaries(PrismaticMeshInput& input)
{
    // A shell input face has three input vertices and at least one incident band cell.
    // get_faces() visits it once even when the sheet has band cells on both sides.
    std::map<Edge, size_t> face_counts;
    for (const auto& face : input.mesh->get_faces()) {
        const auto vertices = input.mesh->get_face_vids(face);
        if (!std::all_of(vertices.begin(), vertices.end(), [&](size_t v) {
                return input.vertex_tags.at(v) == 1;
            }))
            continue;
        bool touches_band = input.offset_tet_tags.at(face.tid(*input.mesh)) == 1;
        if (const auto other = face.switch_tetrahedron(*input.mesh))
            touches_band = touches_band || input.offset_tet_tags.at(other->tid(*input.mesh)) == 1;
        if (!touches_band) continue;
        for (size_t j = 0; j < 3; ++j)
            ++face_counts[edge_key(vertices[j], vertices[(j + 1) % 3])];
    }

    std::set<Edge> boundary_edges;
    std::set<size_t> boundary_vertices, junction_vertices;
    for (const auto& [edge, count] : face_counts) {
        if (count >= 3) junction_vertices.insert(edge.begin(), edge.end());
        if (count != 1 || is_volume_edge(input, edge)) continue;
        boundary_edges.insert(edge);
        boundary_vertices.insert(edge.begin(), edge.end());
    }
    // Include endpoint stars: tau^3 configurations have only one input vertex and
    // therefore cannot contain the entire input edge. Freeze candidates before unmarking.
    std::set<size_t> candidates;
    for (size_t v : boundary_vertices)
        for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(v))
            if (input.offset_tet_tags.at(tid) == 1 && input.input_cells.at(tid) != 1)
                candidates.insert(tid);

    OpenBoundaryStatistics stats{boundary_edges.size(), candidates.size(), 0};
    stats.junction_vertices = junction_vertices.size();
    for (size_t tid : candidates) {
        const auto tet = input.mesh->oriented_tet_vids(tid);
        // T junctions take priority over every removal rule. Any tet containing a
        // junction input vertex belongs to its protected complete one-ring.
        if (std::any_of(tet.begin(), tet.end(), [&](size_t v) {
                return junction_vertices.count(v);
            })) {
            ++stats.protected_candidate_tets;
            continue;
        }
        std::vector<size_t> input_vertices;
        std::set<size_t> offset_parents;
        size_t offset_count = 0;
        for (size_t v : tet) {
            if (input.vertex_tags.at(v) == 1)
                input_vertices.push_back(v);
            else if (input.vertex_tags.at(v) == 2) {
                const auto parent = input.corr_input_vertex.at(v);
                if (parent < 0 || parent >= input.vertices.rows() ||
                    input.vertex_tags.at(parent) != 1 ||
                    input.source_vertex_ids.at(parent) != input.corr_input_vid.at(v))
                    log_and_throw_error("Invalid correspondence in open boundary tet {}.", tid);
                offset_parents.insert(static_cast<size_t>(parent));
                ++offset_count;
            }
        }
        // The six types in Section 4.2 require vertices on both input and offset layers.
        // Other/background configurations are outside this shell classification.
        if (input_vertices.empty() || input_vertices.size() + offset_count != 4) continue;
        const size_t distinct = offset_parents.size();
        bool unmark = (offset_count == 2 || offset_count == 3) && distinct == 1;
        if (offset_count == 3 && distinct == 2) {
            const Edge mapped = edge_key(*offset_parents.begin(), *offset_parents.rbegin());
            const bool input_edge = face_counts.count(mapped) || is_volume_edge(input, mapped);
            unmark = !input_edge || boundary_edges.count(mapped);
        } else if (offset_count == 2 && distinct == 2) {
            const Edge in = edge_key(input_vertices[0], input_vertices[1]);
            const Edge mapped = edge_key(*offset_parents.begin(), *offset_parents.rbegin());
            unmark = in == mapped && boundary_edges.count(in);
        }
        if (unmark) {
            input.offset_tet_tags.at(tid) = -1;
            ++stats.unmarked_tets;
        }
    }
    logger().info(
        "Open boundary: {} input edges, {} candidate shell tetrahedra, {} unmarked as background; "
        "{} junction vertices protect {} candidate tetrahedra",
        stats.open_boundary_edges,
        stats.candidate_tets,
        stats.unmarked_tets,
        stats.junction_vertices,
        stats.protected_candidate_tets);
    return stats;
}
} // namespace wmtk::components::prismatic_mesh
