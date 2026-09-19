#include "prismatic_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {

std::optional<Tau22Element> classify_tau22(const PrismaticMeshInput& input, size_t tet_id)
{
    if (tet_id >= input.mesh->tet_capacity()) return std::nullopt;
    const auto tet = input.mesh->tuple_from_tet(tet_id);
    if (!tet.is_valid(*input.mesh)) return std::nullopt;
    std::vector<size_t> in, off;
    for (size_t v : input.mesh->oriented_tet_vids(tet_id)) {
        if (input.vertex_tags.at(v) == 1)
            in.push_back(v);
        else if (input.vertex_tags.at(v) == 2)
            off.push_back(v);
    }
    if (in.size() != 2 || off.size() != 2) return std::nullopt;
    std::sort(in.begin(), in.end());
    // Input vertices store corr_input_vid=-1. Their correspondence identity is their own
    // original vid, not that sentinel and not their potentially different mesh row index.
    const int64_t a = input.source_vertex_ids.at(in[0]);
    const int64_t b = input.source_vertex_ids.at(in[1]);
    if (a == b || input.corr_input_vid.at(off[0]) == input.corr_input_vid.at(off[1])) {
        return std::nullopt;
    }
    if (input.corr_input_vid.at(off[0]) == b) std::swap(off[0], off[1]);
    if (input.corr_input_vid.at(off[0]) != a || input.corr_input_vid.at(off[1]) != b) {
        return std::nullopt;
    }
    return Tau22Element{{in[0], in[1]}, {off[0], off[1]}};
}

namespace {
using Tet = std::array<size_t, 4>;
Tet sorted(Tet tet)
{
    std::sort(tet.begin(), tet.end());
    return tet;
}
void validate_threshold(double threshold)
{
    if (!std::isfinite(threshold) || threshold < 0) {
        log_and_throw_error("min_tet_volume must be finite and nonnegative.");
    }
}
std::unique_ptr<TetMesh> make_mesh(size_t n, const MatrixXi& cells)
{
    std::vector<Tet> tets(cells.rows());
    for (size_t i = 0; i < tets.size(); ++i) {
        for (int j = 0; j < 4; ++j) tets[i][j] = static_cast<size_t>(cells(i, j));
    }
    auto mesh = std::make_unique<TetMesh>();
    mesh->init_with_isolated_vertices(n, tets);
    return mesh;
}
} // namespace

bool try_unlock_tau22(PrismaticMeshInput& input, size_t tet_id, int side, double min_tet_volume)
{
    validate_threshold(min_tet_volume);
    if (side != 0 && side != 1) log_and_throw_error("Unlock side must be 0 or 1.");
    const auto tau = classify_tau22(input, tet_id);
    if (!tau) return false;
    const size_t u = tau->input_vertices[side];
    const size_t v = tau->offset_vertices[1 - side];
    const size_t destination = tau->offset_vertices[side];
    const int64_t cid = input.vertex_component_ids.at(destination);
    if (cid < 0 || static_cast<size_t>(cid) >= input.offset_components.size() ||
        input.offset_components[cid].input_vertex != u ||
        input.corr_input_vertex.at(destination) != static_cast<int64_t>(u))
        return false;
    const Vector3d midpoint =
        0.5 * input.vertices.row(u).transpose() + 0.5 * input.vertices.row(v).transpose();
    if (!midpoint.allFinite()) return false;
    const size_t x = input.vertices.rows();

    // Operate on a private mesh: even a successful split is invisible unless the directed
    // collapse also succeeds. The original positions/attributes never need rollback.
    PrismaticMeshInput candidate;
    candidate.vertices = input.vertices;
    candidate.vertices.conservativeResize(x + 1, 3);
    candidate.vertices.row(x) = midpoint.transpose();
    std::map<Tet, size_t> parents;
    const auto live = input.mesh->get_tets();
    candidate.tetrahedra.resize(live.size(), 4);
    for (size_t i = 0; i < live.size(); ++i) {
        const size_t tid = live[i].tid(*input.mesh);
        const auto tet = input.mesh->oriented_tet_vids(tid);
        parents.emplace(sorted(tet), tid);
        for (int j = 0; j < 4; ++j) candidate.tetrahedra(i, j) = static_cast<int>(tet[j]);
        if (std::find(tet.begin(), tet.end(), u) != tet.end() &&
            std::find(tet.begin(), tet.end(), v) != tet.end() &&
            !tet_volume_above_threshold(input.vertices, tet, min_tet_volume))
            return false;
    }
    // x must be the newly allocated slot, not a pre-existing isolated vertex.
    candidate.mesh = make_mesh(x, candidate.tetrahedra);
    const auto split_edge = candidate.mesh->tuple_from_edge({u, v});
    candidate.mesh->ensure_free_vert_capacity(1);
    // Splitting an edge can add at most one tet per existing tet.
    candidate.mesh->ensure_free_tet_capacity(live.size());
    std::vector<TetMesh::Tuple> new_edges;
    if (!candidate.mesh->split_edge(split_edge, new_edges)) return false;
    if (candidate.mesh->vert_capacity() != x + 1) return false;

    // Each split child inherits its original tetrahedron's tags. Match by replacing x
    // with the missing endpoint; unchanged tets simply retain their own parent key.
    const auto split_tets = candidate.mesh->get_tets();
    MatrixXi cells(split_tets.size(), 4);
    for (size_t i = 0; i < split_tets.size(); ++i) {
        const auto tet = candidate.mesh->oriented_tet_vids(split_tets[i]);
        Tet parent = tet;
        const auto new_vertex = std::find(parent.begin(), parent.end(), x);
        if (new_vertex != parent.end()) {
            if (!tet_volume_above_threshold(candidate.vertices, tet, min_tet_volume)) return false;
            *new_vertex = std::find(tet.begin(), tet.end(), u) != tet.end() ? v : u;
        }
        const auto original = parents.find(sorted(parent));
        if (original == parents.end()) return false;
        candidate.input_cells.push_back(input.input_cells.at(original->second));
        candidate.offset_tet_tags.push_back(input.offset_tet_tags.at(original->second));
        for (int j = 0; j < 4; ++j) cells(i, j) = static_cast<int>(tet[j]);
    }
    candidate.tetrahedra = std::move(cells);
    candidate.mesh = make_mesh(x + 1, candidate.tetrahedra);

    // x temporarily inherits the DESTINATION's correspondence/component. These attributes
    // exist solely for the checked x->destination collapse; no new source vid is published.
    candidate.source_vertex_ids = input.source_vertex_ids;
    candidate.source_vertex_ids.push_back(-1); // temporary; never exported
    candidate.vertex_tags = input.vertex_tags;
    candidate.vertex_tags.push_back(2);
    candidate.vertex_component_ids = input.vertex_component_ids;
    candidate.vertex_component_ids.push_back(cid);
    candidate.corr_input_vid = input.corr_input_vid;
    candidate.corr_input_vid.push_back(input.source_vertex_ids[u]);
    candidate.corr_input_vertex = input.corr_input_vertex;
    candidate.corr_input_vertex.push_back(static_cast<int64_t>(u));
    candidate.singular_vertex_tags = input.singular_vertex_tags;
    candidate.singular_vertex_tags.push_back(input.singular_vertex_tags.at(destination));
    candidate.offset_components = input.offset_components;
    candidate.offset_components[cid].vertices.push_back(x);
    candidate.offset_vertices = input.offset_vertices;
    candidate.offset_vertices.push_back(x);
    candidate.input_to_offset_vertices = input.input_to_offset_vertices;
    candidate.input_to_offset_vertices.resize(x + 1);
    candidate.input_to_offset_vertices[u].push_back(x);
    if (!try_collapse_offset_edge(candidate, x, destination, min_tet_volume)) return false;
    if (!candidate.mesh->get_one_ring_tids_for_vertex(x).empty()) return false;

    // The target tau22 must disappear. All original vertices (including the destination)
    // retain their exact positions and IDs. Discard the temporary row/slot altogether.
    const Tet target = sorted(input.mesh->oriented_tet_vids(tet_id));
    for (const auto& t : candidate.mesh->get_tets()) {
        const auto tet = candidate.mesh->oriented_tet_vids(t);
        if (std::find(tet.begin(), tet.end(), x) != tet.end()) return false;
        if (sorted(tet) == target) return false;
    }
    auto committed_mesh = make_mesh(x, candidate.tetrahedra);
    input.mesh = std::move(committed_mesh);
    input.tetrahedra = std::move(candidate.tetrahedra);
    input.input_cells = std::move(candidate.input_cells);
    input.offset_tet_tags = std::move(candidate.offset_tet_tags);
    input.offset_face_tags.clear();
    return true;
}

UnlockStatistics unlock_tau22(PrismaticMeshInput& input, double min_tet_volume)
{
    validate_threshold(min_tet_volume);
    UnlockStatistics stats;
    std::vector<Tet> candidates;
    for (const auto& t : input.mesh->get_tets()) {
        if (classify_tau22(input, t.tid(*input.mesh))) {
            candidates.push_back(sorted(input.mesh->oriented_tet_vids(t)));
        }
    }
    stats.candidates = candidates.size();
    // Store vertex keys, never stale tet IDs/tuples across successful unlocks.
    std::map<Tet, size_t> current;
    auto refresh = [&]() {
        current.clear();
        for (const auto& t : input.mesh->get_tets()) {
            current.emplace(sorted(input.mesh->oriented_tet_vids(t)), t.tid(*input.mesh));
        }
    };
    refresh();
    for (const auto& key : candidates) {
        const auto found = current.find(key);
        if (found == current.end()) continue;
        const size_t tid = found->second;
        if (!classify_tau22(input, tid)) continue;
        ++stats.attempted;
        if (try_unlock_tau22(input, tid, 0, min_tet_volume) ||
            try_unlock_tau22(input, tid, 1, min_tet_volume)) {
            ++stats.unlocked;
            refresh();
        }
    }
    for (const auto& t : input.mesh->get_tets()) {
        if (classify_tau22(input, t.tid(*input.mesh))) ++stats.remaining;
    }
    return stats;
}

} // namespace wmtk::components::prismatic_mesh
