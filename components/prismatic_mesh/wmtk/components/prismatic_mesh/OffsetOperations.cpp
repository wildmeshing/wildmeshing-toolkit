#include "OffsetOperations.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <tuple>

namespace wmtk::components::prismatic_mesh {
namespace {

using Edge = std::array<size_t, 2>;

Edge sorted_edge(size_t a, size_t b)
{
    if (a > b) std::swap(a, b);
    return {{a, b}};
}

} // namespace

OpenBoundaryStats PrismOffsetTetMesh::handle_open_boundaries(
    const std::vector<std::array<size_t, 3>>& annotated_faces)
{
    OpenBoundaryStats stats;
    std::map<Edge, size_t> face_count;
    for (const auto& face : annotated_faces) {
        ++face_count[sorted_edge(face[0], face[1])];
        ++face_count[sorted_edge(face[1], face[2])];
        ++face_count[sorted_edge(face[2], face[0])];
    }

    std::set<Edge> boundary_edges;
    for (const auto& [edge, count] : face_count) {
        if (count <= 1) boundary_edges.insert(edge);
    }
    stats.boundary_edges = boundary_edges.size();
    if (boundary_edges.empty()) return stats;

    std::vector<size_t> remove;
    for (const Tuple& tet : get_tets()) {
        const size_t tid = tet.tid(*this);
        if (m_tet_attribute[tid].label != 2) continue;

        std::vector<size_t> input_vertices;
        std::vector<size_t> offset_vertices;
        std::set<size_t> all_sources;
        bool has_unknown_source = false;
        for (const size_t vertex : oriented_tet_vids(tet)) {
            const auto& attributes = m_vertex_attribute[vertex];
            if (attributes.source_vid < 0) {
                has_unknown_source = true;
                break;
            }
            all_sources.insert(static_cast<size_t>(attributes.source_vid));
            if (attributes.label == 1)
                input_vertices.push_back(vertex);
            else if (attributes.label == 2)
                offset_vertices.push_back(vertex);
        }
        if (has_unknown_source || input_vertices.empty() || offset_vertices.empty()) continue;

        bool near_boundary = false;
        for (const Edge& edge : boundary_edges) {
            if (all_sources.count(edge[0]) != 0 && all_sources.count(edge[1]) != 0) {
                near_boundary = true;
                break;
            }
        }
        if (!near_boundary) continue;
        ++stats.candidate_tets;

        const size_t n = offset_vertices.size();
        std::set<size_t> offset_sources;
        for (const size_t vertex : offset_vertices) {
            offset_sources.insert(static_cast<size_t>(m_vertex_attribute[vertex].source_vid));
        }
        const size_t m = offset_sources.size();

        bool should_remove = (n == 2 && m == 1) || (n == 3 && m == 1);
        if (n == 2 && m == 2 && input_vertices.size() == 2) {
            std::set<size_t> input_sources;
            for (const size_t vertex : input_vertices) {
                input_sources.insert(static_cast<size_t>(m_vertex_attribute[vertex].source_vid));
            }
            if (input_sources == offset_sources && input_sources.size() == 2) {
                const Edge corresponding =
                    sorted_edge(*input_sources.begin(), *input_sources.rbegin());
                should_remove = boundary_edges.count(corresponding) != 0;
            }
        } else if (n == 3 && m == 2) {
            const Edge corresponding =
                sorted_edge(*offset_sources.begin(), *offset_sources.rbegin());
            should_remove = boundary_edges.count(corresponding) != 0;
        }

        if (should_remove) remove.push_back(tid);
    }

    std::sort(remove.begin(), remove.end());
    remove.erase(std::unique(remove.begin(), remove.end()), remove.end());
    stats.removed_tets = remove.size();
    if (!remove.empty()) {
        remove_tets_by_ids(remove);
        consolidate_mesh();
    }
    return stats;
}

bool PrismOffsetTetMesh::is_offset_surface_edge(const Tuple& edge) const
{
    const size_t v0 = edge.vid(*this);
    const size_t v1 = edge.switch_vertex(*this).vid(*this);
    if (m_vertex_attribute[v0].label != 2 || m_vertex_attribute[v1].label != 2) {
        return false;
    }

    // An outer offset-surface edge belongs to a face separating an offset tet
    // from a non-offset tet (or the ambient mesh boundary). Deriving this from
    // tet labels avoids relying on stale local face ids after an edit.
    for (const Tuple& tet : get_incident_tets_for_edge(edge)) {
        if (m_tet_attribute[tet.tid(*this)].label != 2) continue;
        const auto vids = oriented_tet_vids(tet);
        for (size_t omitted = 0; omitted < 4; ++omitted) {
            std::array<size_t, 3> face;
            size_t k = 0;
            bool has_v0 = false;
            bool has_v1 = false;
            for (size_t i = 0; i < 4; ++i) {
                if (i == omitted) continue;
                face[k++] = vids[i];
                has_v0 = has_v0 || vids[i] == v0;
                has_v1 = has_v1 || vids[i] == v1;
            }
            if (!has_v0 || !has_v1) continue;
            const auto [face_tuple, unused] = tuple_from_face(face);
            const auto adjacent = face_tuple.switch_tetrahedron(*this);
            if (!adjacent || m_tet_attribute[adjacent->tid(*this)].label != 2) {
                return true;
            }
        }
    }
    return false;
}

bool PrismOffsetTetMesh::classify_tau_2_2(const Tuple& tet, Tau22& result) const
{
    if (m_tet_attribute[tet.tid(*this)].label > 2) return false;
    std::map<int64_t, size_t> input_by_source;
    std::map<int64_t, size_t> offset_by_source;
    for (const size_t vertex : oriented_tet_vids(tet)) {
        const auto& attributes = m_vertex_attribute[vertex];
        if (attributes.source_vid < 0) return false;
        if (attributes.label == 1) {
            if (!input_by_source.emplace(attributes.source_vid, vertex).second) return false;
        } else if (attributes.label == 2) {
            if (!offset_by_source.emplace(attributes.source_vid, vertex).second) return false;
        } else {
            return false;
        }
    }
    if (input_by_source.size() != 2 || offset_by_source.size() != 2) return false;
    if (input_by_source.begin()->first != offset_by_source.begin()->first ||
        input_by_source.rbegin()->first != offset_by_source.rbegin()->first) {
        return false;
    }

    size_t i = 0;
    for (const auto& [source, input] : input_by_source) {
        result.sources[i] = source;
        result.input[i] = input;
        result.offset[i] = offset_by_source.at(source);
        ++i;
    }
    return true;
}

size_t PrismOffsetTetMesh::count_equal_source_edges() const
{
    size_t count = 0;
    for (const Tuple& edge : get_edges()) {
        const size_t v0 = edge.vid(*this);
        const size_t v1 = edge.switch_vertex(*this).vid(*this);
        const auto& a0 = m_vertex_attribute[v0];
        const auto& a1 = m_vertex_attribute[v1];
        count +=
            is_offset_surface_edge(edge) && a0.source_vid >= 0 && a0.source_vid == a1.source_vid;
    }
    return count;
}

size_t PrismOffsetTetMesh::count_tau_2_2() const
{
    size_t count = 0;
    for (const Tuple& tet : get_tets()) {
        Tau22 configuration;
        count += classify_tau_2_2(tet, configuration);
    }
    return count;
}

std::set<int64_t> PrismOffsetTetMesh::non_bijective_sources() const
{
    std::set<int64_t> result;
    for (const size_t vertex : non_bijective_vertices()) {
        const int64_t source = m_vertex_attribute[vertex].source_vid;
        if (source >= 0) result.insert(source);
    }
    return result;
}

std::set<size_t> PrismOffsetTetMesh::non_bijective_vertices() const
{
    std::set<size_t> result;
    for (const Tuple& edge : get_edges()) {
        const size_t v0 = edge.vid(*this);
        const size_t v1 = edge.switch_vertex(*this).vid(*this);
        const auto& a0 = m_vertex_attribute[v0];
        const auto& a1 = m_vertex_attribute[v1];
        if (is_offset_surface_edge(edge) && a0.source_vid >= 0 && a0.source_vid == a1.source_vid) {
            // Algorithm 9 marks every vertex of an offset triangle incident to
            // a residual equal-source edge, not every vertex elsewhere that
            // happens to share the same input source.
            for (const Tuple& face : get_faces()) {
                const auto vertices = get_face_vids(face);
                const bool contains_edge =
                    std::find(vertices.begin(), vertices.end(), v0) != vertices.end() &&
                    std::find(vertices.begin(), vertices.end(), v1) != vertices.end();
                if (!contains_edge) continue;
                bool all_offset = true;
                for (const size_t vertex : vertices) {
                    all_offset = all_offset && m_vertex_attribute[vertex].label == 2;
                }
                if (all_offset) result.insert(vertices.begin(), vertices.end());
            }
        }
    }
    for (const Tuple& tet : get_tets()) {
        Tau22 configuration;
        if (!classify_tau_2_2(tet, configuration)) continue;
        result.insert(configuration.offset.begin(), configuration.offset.end());
    }
    return result;
}

bool PrismOffsetTetMesh::collapse_edge_before(const Tuple& edge)
{
    if (m_collapse_kind == CollapseKind::None || !edge.is_valid(*this)) return false;
    const size_t removed = edge.vid(*this);
    const size_t survivor = edge.switch_vertex(*this).vid(*this);
    if (removed != m_expected_removed || survivor != m_expected_survivor) return false;

    const auto& a_removed = m_vertex_attribute[removed];
    const auto& a_survivor = m_vertex_attribute[survivor];
    if (m_collapse_kind == CollapseKind::EqualSource) {
        if (a_removed.label != 2 || a_survivor.label != 2 || a_removed.source_vid < 0 ||
            a_removed.source_vid != a_survivor.source_vid || !is_offset_surface_edge(edge)) {
            return false;
        }
    } else {
        if (a_removed.source_vid != -1 || a_survivor.label != 1 || a_survivor.source_vid < 0) {
            return false;
        }
    }
    m_survivor_attributes = a_survivor;
    return true;
}

bool PrismOffsetTetMesh::collapse_edge_after(const Tuple& survivor)
{
    if (survivor.vid(*this) != m_expected_survivor) return false;
    m_vertex_attribute[survivor.vid(*this)] = m_survivor_attributes;
    return true;
}

size_t PrismOffsetTetMesh::collapse_equal_source_edges(OffsetOperationStats* stats)
{
    struct Candidate
    {
        int64_t source;
        Edge edge;
    };
    std::vector<Candidate> candidates;
    for (const Tuple& edge : get_edges()) {
        const size_t v0 = edge.vid(*this);
        const size_t v1 = edge.switch_vertex(*this).vid(*this);
        const auto& a0 = m_vertex_attribute[v0];
        const auto& a1 = m_vertex_attribute[v1];
        if (is_offset_surface_edge(edge) && a0.source_vid >= 0 && a0.source_vid == a1.source_vid) {
            candidates.push_back({a0.source_vid, sorted_edge(v0, v1)});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return std::tie(a.source, a.edge) < std::tie(b.source, b.edge);
    });

    size_t successes = 0;
    for (const Candidate& candidate : candidates) {
        Tuple edge = tuple_from_edge(candidate.edge);
        if (!edge.is_valid(*this)) continue;
        const size_t v0 = edge.vid(*this);
        const size_t v1 = edge.switch_vertex(*this).vid(*this);
        if (stats) ++stats->equal_source_attempts;

        // TetMesh removes the tuple's current vertex. Stable ids make the
        // survivor deterministic without changing the paper's admissibility.
        const size_t first_survivor = std::min(v0, v1);
        const size_t first_removed = std::max(v0, v1);
        const auto attempt = [&](const size_t removed, const size_t survivor) {
            Tuple oriented = tuple_from_edge(candidate.edge);
            if (!oriented.is_valid(*this)) return false;
            if (oriented.vid(*this) != removed) oriented = oriented.switch_vertex(*this);
            m_collapse_kind = CollapseKind::EqualSource;
            m_expected_removed = removed;
            m_expected_survivor = survivor;
            std::vector<Tuple> new_edges;
            const bool result = collapse_edge(oriented, new_edges);
            m_collapse_kind = CollapseKind::None;
            return result;
        };
        // Stable-id direction first. If geometry rejects that direction, the
        // opposite direction is a deterministic second admissible choice.
        if (attempt(first_removed, first_survivor) || attempt(first_survivor, first_removed)) {
            ++successes;
            if (stats) ++stats->equal_source_collapses;
        }
    }
    return successes;
}

PrismOffsetTetMesh::Snapshot PrismOffsetTetMesh::snapshot() const
{
    Snapshot state;
    state.vertex_count = vert_capacity();
    for (const Tuple& tet : get_tets()) state.tets.push_back(oriented_tet_vids(tet));
    state.vertices = m_vertex_attribute.m_attributes;
    state.edges = m_edge_attribute.m_attributes;
    state.faces = m_face_attribute.m_attributes;
    state.tet_attributes = m_tet_attribute.m_attributes;
    return state;
}

void PrismOffsetTetMesh::restore(const Snapshot& state)
{
    init_with_isolated_vertices(state.vertex_count, state.tets);
    m_vertex_attribute.m_attributes = state.vertices;
    m_edge_attribute.m_attributes = state.edges;
    m_face_attribute.m_attributes = state.faces;
    m_tet_attribute.m_attributes = state.tet_attributes;
}

bool PrismOffsetTetMesh::try_unlock(const Tau22& configuration, const size_t diagonal)
{
    // For target input i, split the opposite cross-layer diagonal joining the
    // other input vertex to offset(i), then collapse the new vertex to input(i).
    const size_t target_index = diagonal;
    const size_t other_index = 1 - diagonal;
    Tuple split_diagonal =
        tuple_from_edge({{configuration.input[other_index], configuration.offset[target_index]}});
    if (!split_diagonal.is_valid(*this) || split_diagonal.is_boundary_edge(*this)) return false;

    const size_t tau_before = count_tau_2_2();
    const Snapshot before = snapshot();
    std::vector<Tuple> split_edges;
    m_edge_split_mode = EdgeSplitMode::Midpoint;
    if (!split_edge(split_diagonal, split_edges)) {
        restore(before);
        return false;
    }
    const size_t new_vertex = vert_capacity() - 1;
    Tuple collapse = tuple_from_edge({{new_vertex, configuration.input[target_index]}});
    if (!collapse.is_valid(*this)) {
        restore(before);
        return false;
    }
    if (collapse.vid(*this) != new_vertex) collapse = collapse.switch_vertex(*this);

    m_collapse_kind = CollapseKind::Unlock;
    m_expected_removed = new_vertex;
    m_expected_survivor = configuration.input[target_index];
    std::vector<Tuple> new_edges;
    const bool success = collapse_edge(collapse, new_edges);
    m_collapse_kind = CollapseKind::None;
    // The composite is an unlocking operation only if it strictly reduces the
    // paper's obstruction count. This prevents a split-collapse cycle from
    // moving an equivalent tau^2_2 tetrahedron around indefinitely.
    if (!success || !check_mesh_connectivity_validity() || count_tau_2_2() >= tau_before) {
        restore(before);
        return false;
    }
    return true;
}

size_t PrismOffsetTetMesh::unlock_tau_2_2(OffsetOperationStats* stats)
{
    std::vector<Tau22> candidates;
    for (const Tuple& tet : get_tets()) {
        Tau22 configuration;
        if (classify_tau_2_2(tet, configuration)) candidates.push_back(configuration);
    }
    std::sort(candidates.begin(), candidates.end(), [](const Tau22& a, const Tau22& b) {
        return std::tie(a.sources, a.input, a.offset) < std::tie(b.sources, b.input, b.offset);
    });

    size_t successes = 0;
    for (const Tau22& candidate : candidates) {
        // A prior edit may have consumed this tetrahedron. Reclassify by the four
        // vertices before attempting either diagonal.
        bool still_present = true;
        for (size_t i = 0; i < 2; ++i) {
            still_present = still_present &&
                            tuple_from_vertex(candidate.input[i]).is_valid(*this) &&
                            tuple_from_vertex(candidate.offset[i]).is_valid(*this);
        }
        if (!still_present) continue;
        if (stats) ++stats->unlock_attempts;
        bool success = try_unlock(candidate, 0);
        if (!success) {
            if (stats) ++stats->unlock_rollbacks;
            success = try_unlock(candidate, 1);
        }
        if (success) {
            ++successes;
            if (stats) ++stats->unlocks;
        } else if (stats) {
            ++stats->unlock_rollbacks;
        }
    }
    return successes;
}

OffsetOperationStats PrismOffsetTetMesh::simplify_offset(const size_t max_iterations)
{
    OffsetOperationStats stats;
    for (size_t iteration = 0; iteration < max_iterations; ++iteration) {
        const size_t collapsed = collapse_equal_source_edges(&stats);
        const size_t unlocked = unlock_tau_2_2(&stats);
        ++stats.iterations;
        if (collapsed + unlocked == 0) break;
        consolidate_mesh();
    }
    stats.remaining_equal_source_edges = count_equal_source_edges();
    stats.remaining_tau_2_2 = count_tau_2_2();
    return stats;
}

} // namespace wmtk::components::prismatic_mesh
