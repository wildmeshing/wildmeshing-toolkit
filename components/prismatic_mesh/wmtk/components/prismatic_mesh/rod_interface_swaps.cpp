#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <wmtk/utils/Logger.hpp>
#include "junction_protection.hpp"
#include "rod_mesh.hpp"

namespace wmtk::components::prismatic_mesh {
namespace {
using Tri = std::array<size_t, 3>;
using Tet = std::array<size_t, 4>;
using Decomposition = std::vector<Tet>;
template <class T>
T sorted(T value)
{
    std::sort(value.begin(), value.end());
    return value;
}
void require(bool valid, const std::string& message)
{
    if (!valid) throw std::runtime_error("rod interface swap: " + message);
}
Tet source_tet(const PrismaticMeshInput& input, size_t row)
{
    return {
        size_t(input.tetrahedra(row, 0)),
        size_t(input.tetrahedra(row, 1)),
        size_t(input.tetrahedra(row, 2)),
        size_t(input.tetrahedra(row, 3))};
}
std::array<Tri, 4> tet_faces(const Tet& t)
{
    return {{{t[1], t[2], t[3]}, {t[0], t[3], t[2]}, {t[0], t[1], t[3]}, {t[0], t[2], t[1]}}};
}
bool same_orientation(Tri a, Tri b)
{
    std::rotate(a.begin(), std::min_element(a.begin(), a.end()), a.end());
    std::rotate(b.begin(), std::min_element(b.begin(), b.end()), b.end());
    return a == b;
}
std::map<Tri, Tri> layer_boundary(const Decomposition& cells)
{
    std::map<Tri, std::vector<Tri>> owners;
    for (const auto& t : cells)
        for (const auto& face : tet_faces(t)) owners[sorted(face)].push_back(face);
    std::map<Tri, Tri> result;
    for (const auto& [key, faces] : owners) {
        require(faces.size() <= 2, "nonmanifold cavity");
        if (faces.size() == 1)
            result[key] = faces[0];
        else
            require(!same_orientation(faces[0], faces[1]), "inconsistent cavity orientation");
    }
    return result;
}
using SwapEdge = std::array<size_t, 2>;

struct InterfaceSwapPartition
{
    std::vector<size_t> old_rows;
    std::vector<size_t> link;
    Decomposition replacement;
    int input_tag = 0, offset_tag = -1;
};

bool split_interface_star(
    const PrismaticMeshInput& input,
    const SwapEdge& edge,
    const SwapEdge& ends,
    double floor,
    InterfaceSwapPartition& part,
    std::string& reason)
{
    std::map<size_t, std::set<size_t>> adjacency;
    for (size_t tid : part.old_rows) {
        const auto t = source_tet(input, tid);
        std::vector<size_t> opposite;
        for (size_t v : t)
            if (v != edge[0] && v != edge[1]) opposite.push_back(v);
        if (opposite.size() != 2) {
            reason = "invalid_edge_star";
            return false;
        }
        adjacency[opposite[0]].insert(opposite[1]);
        adjacency[opposite[1]].insert(opposite[0]);
    }
    std::set<size_t> endpoints;
    for (const auto& [v, ns] : adjacency) {
        if (ns.size() == 1)
            endpoints.insert(v);
        else if (ns.size() != 2) {
            reason = "non_path_volume_link";
            return false;
        }
    }
    if (endpoints != std::set<size_t>{ends[0], ends[1]}) {
        reason = "wrong_volume_link_ends";
        return false;
    }
    part.link = {ends[0]};
    size_t previous = std::numeric_limits<size_t>::max(), current = ends[0];
    while (current != ends[1]) {
        const auto& neighbors = adjacency.at(current);
        const auto next = std::find_if(neighbors.begin(), neighbors.end(), [&](size_t v) {
            return v != previous;
        });
        if (next == neighbors.end() || part.link.size() > adjacency.size()) {
            reason = "disconnected_volume_link";
            return false;
        }
        previous = current;
        current = *next;
        part.link.push_back(current);
    }
    if (part.link.size() != adjacency.size()) {
        reason = "disconnected_volume_link";
        return false;
    }
    if (!tet_volume_above_threshold(
            input.vertices,
            {edge[0], edge[1], part.link[0], part.link[1]},
            0))
        std::reverse(part.link.begin(), part.link.end());
    for (size_t i = 1; i < part.link.size(); ++i)
        if (!tet_volume_above_threshold(
                input.vertices,
                {edge[0], edge[1], part.link[i - 1], part.link[i]},
                0)) {
            reason = "inconsistent_volume_link";
            return false;
        }
    const size_t n = part.link.size();
    struct State
    {
        bool valid = false;
        double score = -1;
        Decomposition tets;
    };
    std::vector<std::vector<State>> dp(n, std::vector<State>(n));
    for (size_t i = 0; i + 1 < n; ++i) {
        dp[i][i + 1].valid = true;
        dp[i][i + 1].score = std::numeric_limits<double>::infinity();
    }
    auto determinant = [&](const Tet& t) {
        const Vector3d a = input.vertices.row(t[0]);
        Eigen::Matrix3d M;
        for (int j = 0; j < 3; ++j) M.col(j) = Vector3d(input.vertices.row(t[j + 1])) - a;
        return M.determinant();
    };
    // Triangulate the open edge-link polygon after replacing the old diagonal.
    // Every polygon triangle is coned to the old edge's two endpoints. The two
    // cones have opposite winding; both must be strictly positive as stored.
    for (size_t width = 2; width < n; ++width)
        for (size_t i = 0; i + width < n; ++i) {
            const size_t j = i + width;
            for (size_t k = i + 1; k < j; ++k) {
                if (!dp[i][k].valid || !dp[k][j].valid) continue;
                const Tet a = {edge[0], part.link[i], part.link[k], part.link[j]};
                const Tet b = {edge[1], part.link[i], part.link[j], part.link[k]};
                if (!tet_volume_above_threshold(input.vertices, a, floor) ||
                    !tet_volume_above_threshold(input.vertices, b, floor))
                    continue;
                const double score =
                    std::min({dp[i][k].score, dp[k][j].score, determinant(a), determinant(b)});
                if (dp[i][j].valid && dp[i][j].score >= score) continue;
                auto& result = dp[i][j];
                result.valid = true;
                result.score = score;
                result.tets = dp[i][k].tets;
                result.tets.insert(result.tets.end(), dp[k][j].tets.begin(), dp[k][j].tets.end());
                result.tets.push_back(a);
                result.tets.push_back(b);
            }
        }
    if (!dp[0][n - 1].valid) {
        reason = "no_positive_volume_star_swap";
        return false;
    }
    part.replacement = std::move(dp[0][n - 1].tets);
    part.input_tag = input.input_cells[part.old_rows.front()];
    part.offset_tag = input.offset_tet_tags[part.old_rows.front()];
    for (size_t tid : part.old_rows)
        if (input.input_cells[tid] != part.input_tag ||
            input.offset_tet_tags[tid] != part.offset_tag) {
            reason = "mixed_volume_star_regions";
            return false;
        }
    return true;
}

bool try_rod_interface_swap(
    PrismaticMeshInput& input,
    const SwapEdge& old_edge,
    const SwapEdge& new_edge,
    double floor,
    nlohmann::json& operation,
    std::string& reason)
{
    if (input.rod_state->frozen_offset.count(old_edge[0]) ||
        input.rod_state->frozen_offset.count(old_edge[1]) ||
        input.rod_state->frozen_offset.count(new_edge[0]) ||
        input.rod_state->frozen_offset.count(new_edge[1])) {
        reason = "protected_offset";
        return false;
    }
    InterfaceSwapPartition band, background;
    std::set<size_t> removed;
    Decomposition old;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        const auto t = source_tet(input, tid);
        if (std::find(t.begin(), t.end(), old_edge[0]) == t.end() ||
            std::find(t.begin(), t.end(), old_edge[1]) == t.end())
            continue;
        if (input.input_cells[tid] == 1 || junction_tet_protected(input, t)) {
            reason = "protected_volume_star";
            return false;
        }
        (input.offset_tet_tags[tid] == 1 ? band : background).old_rows.push_back(tid);
        removed.insert(tid);
        old.push_back(t);
    }
    if (band.old_rows.empty() || background.old_rows.empty()) {
        reason = "missing_interface_side";
        return false;
    }
    if (!split_interface_star(input, old_edge, new_edge, floor, band, reason)) {
        reason = "band_" + reason;
        return false;
    }
    if (!split_interface_star(input, old_edge, new_edge, floor, background, reason)) {
        reason = "background_" + reason;
        return false;
    }
    Decomposition replacement = band.replacement;
    replacement.insert(
        replacement.end(),
        background.replacement.begin(),
        background.replacement.end());
    const auto before = layer_boundary(old), after = layer_boundary(replacement);
    if (before.size() != after.size()) {
        reason = "swap_cavity_boundary_changed";
        return false;
    }
    for (const auto& [key, f] : before)
        if (!after.count(key) || !same_orientation(f, after.at(key))) {
            reason = "swap_cavity_boundary_changed";
            return false;
        }
    std::set<Tet> all;
    std::map<Tri, std::vector<Tri>> faces;
    Decomposition cells;
    std::vector<int> input_tags, offset_tags;
    auto append = [&](const Tet& t, int region, int offset) {
        if (!all.insert(sorted(t)).second) return false;
        for (const auto& f : tet_faces(t)) {
            auto& owners = faces[sorted(f)];
            if (owners.size() == 2 || (!owners.empty() && same_orientation(f, owners[0])))
                return false;
            owners.push_back(f);
        }
        cells.push_back(t);
        input_tags.push_back(region);
        offset_tags.push_back(offset);
        return true;
    };
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        if (removed.count(tid)) continue;
        if (!append(source_tet(input, tid), input.input_cells[tid], input.offset_tet_tags[tid])) {
            reason = "nonmanifold_or_duplicate_swap";
            return false;
        }
    }
    for (const auto* part : {&band, &background})
        for (const auto& t : part->replacement)
            if (!append(t, part->input_tag, part->offset_tag)) {
                reason = "nonmanifold_or_duplicate_swap";
                return false;
            }
    operation = {
        {"old_edge_vids",
         {input.source_vertex_ids[old_edge[0]], input.source_vertex_ids[old_edge[1]]}},
        {"new_edge_vids",
         {input.source_vertex_ids[new_edge[0]], input.source_vertex_ids[new_edge[1]]}},
        {"old_band_tets", band.old_rows.size()},
        {"new_band_tets", band.replacement.size()},
        {"old_background_tets", background.old_rows.size()},
        {"new_background_tets", background.replacement.size()},
        {"band_link_size", band.link.size()},
        {"background_link_size", background.link.size()}};
    input.tetrahedra.resize(cells.size(), 4);
    for (size_t i = 0; i < cells.size(); ++i)
        for (int j = 0; j < 4; ++j) input.tetrahedra(i, j) = cells[i][j];
    input.input_cells = std::move(input_tags);
    input.offset_tet_tags = std::move(offset_tags);
    return true;
}

bool prepare_background_face_swap(
    PrismaticMeshInput& input,
    const Tri& key,
    double floor,
    nlohmann::json& operation)
{
    std::vector<size_t> owners;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        const auto t = source_tet(input, tid);
        if (std::all_of(key.begin(), key.end(), [&](size_t v) {
                return std::find(t.begin(), t.end(), v) != t.end();
            }))
            owners.push_back(tid);
    }
    if (owners.size() != 2) return false;
    for (size_t tid : owners)
        if (input.offset_tet_tags[tid] == 1 || input.input_cells[tid] == 1 ||
            junction_tet_protected(input, source_tet(input, tid)))
            return false;
    if (input.input_cells[owners[0]] != input.input_cells[owners[1]] ||
        input.offset_tet_tags[owners[0]] != input.offset_tet_tags[owners[1]])
        return false;
    std::array<size_t, 2> opposite;
    Tri face;
    for (int j = 0; j < 2; ++j) {
        const auto t = source_tet(input, owners[j]);
        for (size_t v : t)
            if (std::find(key.begin(), key.end(), v) == key.end()) opposite[j] = v;
        if (j == 0)
            for (const auto& f : tet_faces(t))
                if (sorted(f) == key) face = f;
    }
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        if (tid == owners[0] || tid == owners[1]) continue;
        const auto t = source_tet(input, tid);
        if (std::find(t.begin(), t.end(), opposite[0]) != t.end() &&
            std::find(t.begin(), t.end(), opposite[1]) != t.end())
            return false;
    }
    Decomposition proposal;
    for (int j = 0; j < 3; ++j) {
        const Tet t = {opposite[0], opposite[1], face[j], face[(j + 1) % 3]};
        if (!tet_volume_above_threshold(input.vertices, t, floor)) return false;
        proposal.push_back(t);
    }
    const auto before =
        layer_boundary({source_tet(input, owners[0]), source_tet(input, owners[1])});
    const auto after = layer_boundary(proposal);
    if (before.size() != after.size()) return false;
    for (const auto& [f, orientation] : before)
        if (!after.count(f) || !same_orientation(orientation, after.at(f))) return false;
    Decomposition tets;
    std::vector<int> regions, bands;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        if (tid == owners[0] || tid == owners[1]) continue;
        tets.push_back(source_tet(input, tid));
        regions.push_back(input.input_cells[tid]);
        bands.push_back(input.offset_tet_tags[tid]);
    }
    for (const auto& t : proposal) {
        tets.push_back(t);
        regions.push_back(input.input_cells[owners[0]]);
        bands.push_back(input.offset_tet_tags[owners[0]]);
    }
    operation = {
        {"type", "background_2_to_3"},
        {"old_face_vids",
         {input.source_vertex_ids[face[0]],
          input.source_vertex_ids[face[1]],
          input.source_vertex_ids[face[2]]}},
        {"new_edge_vids",
         {input.source_vertex_ids[opposite[0]], input.source_vertex_ids[opposite[1]]}}};
    input.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i)
        for (int j = 0; j < 4; ++j) input.tetrahedra(i, j) = tets[i][j];
    input.input_cells = std::move(regions);
    input.offset_tet_tags = std::move(bands);
    return true;
}

bool search_rod_interface_swap(
    PrismaticMeshInput& input,
    const SwapEdge& edge,
    const SwapEdge& replacement,
    double floor,
    int depth,
    size_t& attempts,
    nlohmann::json& operation,
    std::string& reason)
{
    if (++attempts > 120) {
        reason = "background_swap_search_limit";
        return false;
    }
    if (try_rod_interface_swap(input, edge, replacement, floor, operation, reason)) return true;
    if (depth == 0 || reason != "background_no_positive_volume_star_swap") return false;
    std::set<Tri> faces;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        if (input.offset_tet_tags[tid] == 1 || input.input_cells[tid] == 1) continue;
        const auto t = source_tet(input, tid);
        if (std::find(t.begin(), t.end(), edge[0]) == t.end() ||
            std::find(t.begin(), t.end(), edge[1]) == t.end())
            continue;
        for (const auto& f : tet_faces(t)) faces.insert(sorted(f));
    }
    for (const auto& face : faces) {
        const auto backup = input.tetrahedra;
        const auto backup_input = input.input_cells, backup_offset = input.offset_tet_tags;
        nlohmann::json preparation;
        if (!prepare_background_face_swap(input, face, floor, preparation)) continue;
        nlohmann::json candidate;
        std::string next_reason;
        if (search_rod_interface_swap(
                input,
                edge,
                replacement,
                floor,
                depth - 1,
                attempts,
                candidate,
                next_reason)) {
            if (!candidate.contains("preparatory_background_swaps"))
                candidate["preparatory_background_swaps"] = nlohmann::json::array();
            candidate["preparatory_background_swaps"].insert(
                candidate["preparatory_background_swaps"].begin(),
                preparation);
            candidate["background_search_attempts"] = attempts;
            operation = std::move(candidate);
            reason.clear();
            return true;
        }
        input.tetrahedra = backup;
        input.input_cells = backup_input;
        input.offset_tet_tags = backup_offset;
        if (attempts > 120) break;
    }
    return false;
}

} // namespace
nlohmann::json align_rod_interfaces_by_swaps(PrismaticMeshInput& input, double floor)
{
    nlohmann::json report = {
        {"method", "target_ring_edges_surface_2_2_swap_with_both_volume_stars"},
        {"vertex_positions_unchanged", true},
        {"operations", nlohmann::json::array()},
        {"layers", nlohmann::json::array()},
        {"swaps", 0},
        {"repaired_layers", 0},
        {"failed_layers", 0},
        {"rejections", nlohmann::json::object()}};
    bool has_background = false;
    for (size_t i = 0; i < input.offset_tet_tags.size(); ++i)
        has_background |= input.offset_tet_tags[i] != 1 && input.input_cells[i] != 1;
    if (!has_background) {
        report["skip_reason"] = "no_background";
        return report;
    }
    auto& state = *input.rod_state;
    for (const auto& axis : state.input_edges) {
        if (!state.vertex_frame.count(axis[0]) || !state.vertex_frame.count(axis[1])) continue;
        const auto& lower = state.frames[state.vertex_frame.at(axis[0])].ring;
        const auto& upper = state.frames[state.vertex_frame.at(axis[1])].ring;
        if (lower.size() < 3 || lower.size() != upper.size()) continue;
        std::set<SwapEdge> target;
        for (size_t k = 0; k < lower.size(); ++k)
            target.insert(sorted(SwapEdge{lower[k], upper[k]}));
        const auto old_tets = input.tetrahedra;
        const auto old_inputs = input.input_cells, old_offsets = input.offset_tet_tags;
        auto operations = nlohmann::json::array();
        bool complete = false, changed = false;
        std::string failure;
        for (size_t iteration = 0; iteration < 4 * lower.size(); ++iteration) {
            Decomposition cavity;
            for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
                if (input.offset_tet_tags[tid] != 1) continue;
                const auto t = source_tet(input, tid);
                std::set<int64_t> parents;
                for (size_t v : t)
                    parents.insert(
                        input.vertex_tags[v] == 1 ? int64_t(v) : input.corr_input_vertex[v]);
                if (parents == std::set<int64_t>{int64_t(axis[0]), int64_t(axis[1])})
                    cavity.push_back(t);
            }
            const auto boundary = layer_boundary(cavity);
            std::map<SwapEdge, std::vector<Tri>> users;
            for (const auto& [key, f] : boundary) {
                if (!std::all_of(key.begin(), key.end(), [&](size_t v) {
                        return input.vertex_tags[v] == 2;
                    }))
                    continue;
                for (int j = 0; j < 3; ++j)
                    users[sorted(SwapEdge{f[j], f[(j + 1) % 3]})].push_back(f);
            }
            std::set<SwapEdge> missing;
            for (const auto& e : target)
                if (!users.count(e)) missing.insert(e);
            if (missing.empty()) {
                complete = true;
                break;
            }
            bool accepted = false;
            for (const auto& [edge, triangles] : users) {
                if (triangles.size() != 2 || target.count(edge)) continue;
                if (input.corr_input_vertex[edge[0]] == input.corr_input_vertex[edge[1]]) continue;
                std::set<size_t> opposite;
                for (const auto& f : triangles)
                    for (size_t v : f)
                        if (v != edge[0] && v != edge[1]) opposite.insert(v);
                if (opposite.size() != 2) continue;
                const SwapEdge replacement = {*opposite.begin(), *opposite.rbegin()};
                if (!missing.count(replacement)) continue;
                nlohmann::json operation;
                std::string reason;
                size_t search_attempts = 0;
                constexpr int search_depth = 3;
                if (!search_rod_interface_swap(
                        input,
                        edge,
                        replacement,
                        floor,
                        search_depth,
                        search_attempts,
                        operation,
                        reason)) {
                    failure = reason;
                    report["rejections"][reason] =
                        report["rejections"].value(reason, size_t(0)) + 1;
                    continue;
                }
                operation["input_vertex_ids"] = {
                    input.source_vertex_ids[axis[0]],
                    input.source_vertex_ids[axis[1]]};
                operations.push_back(std::move(operation));
                accepted = true;
                changed = true;
                break;
            }
            if (!accepted) {
                if (failure.empty()) failure = "no_direct_target_edge_swap";
                break;
            }
        }
        if (!changed && complete) continue;
        if (!complete) {
            input.tetrahedra = old_tets;
            input.input_cells = old_inputs;
            input.offset_tet_tags = old_offsets;
            report["failed_layers"] = report["failed_layers"].get<size_t>() + 1;
        } else {
            report["swaps"] = report["swaps"].get<size_t>() + operations.size();
            report["repaired_layers"] = report["repaired_layers"].get<size_t>() + 1;
            for (const auto& operation : operations) report["operations"].push_back(operation);
        }
        report["layers"].push_back(
            {{"input_vertex_ids",
              {input.source_vertex_ids[axis[0]], input.source_vertex_ids[axis[1]]}},
             {"complete", complete},
             {"swaps", complete ? operations.size() : size_t(0)},
             {"reason", complete ? "" : failure}});
    }
    std::vector<Tet> tets(input.tetrahedra.rows());
    for (size_t i = 0; i < tets.size(); ++i) tets[i] = source_tet(input, i);
    auto mesh = std::make_unique<TetMesh>();
    mesh->init_with_isolated_vertices(input.vertices.rows(), tets);
    require(mesh->check_mesh_connectivity_validity(), "invalid interface-swap tet mesh");
    input.mesh = std::move(mesh);
    label_offset_faces(input);
    for (const auto& tet : tets)
        require(tet_volume_above_threshold(input.vertices, tet, 0), "nonpositive swapped tet");
    layer_boundary(tets);
    logger().info(
        "Rod interface swaps: {} operations, {} layers repaired, {} layers blocked",
        report["swaps"].get<size_t>(),
        report["repaired_layers"].get<size_t>(),
        report["failed_layers"].get<size_t>());
    return report;
}

} // namespace wmtk::components::prismatic_mesh
