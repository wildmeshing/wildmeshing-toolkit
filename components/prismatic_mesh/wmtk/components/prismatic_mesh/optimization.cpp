#include "background_remeshing.hpp"
#include "prismatic_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
constexpr size_t no_vertex = std::numeric_limits<size_t>::max();

void validate_threshold(double threshold)
{
    if (!std::isfinite(threshold) || threshold < 0) {
        log_and_throw_error("min_tet_volume must be finite and nonnegative.");
    }
}

bool volume_above(
    const MatrixXd& vertices,
    const std::array<size_t, 4>& tet,
    double threshold,
    size_t relocated = no_vertex,
    const Vector3d& position = Vector3d::Zero())
{
    std::array<Vector3d, 4> points;
    for (size_t i = 0; i < 4; ++i) {
        points[i] = tet[i] == relocated ? position : Vector3d(vertices.row(tet[i]));
        if (!points[i].allFinite()) return false;
    }
    // Convert coordinates BEFORE subtraction. GMP represents every input double exactly;
    // both the orientation and the strict volume threshold comparison are exact.
    std::array<std::array<Rational, 3>, 3> e;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j)
            e[i][j] = Rational(points[i + 1][j]) - Rational(points[0][j]);
    }
    const Rational determinant = e[0][0] * (e[1][1] * e[2][2] - e[1][2] * e[2][1]) -
                                 e[0][1] * (e[1][0] * e[2][2] - e[1][2] * e[2][0]) +
                                 e[0][2] * (e[1][0] * e[2][1] - e[1][1] * e[2][0]);
    return determinant > Rational(6) * Rational(threshold);
}

bool active(const PrismaticMeshInput& input, size_t v)
{
    return v < input.mesh->vert_capacity() && !input.mesh->get_one_ring_tids_for_vertex(v).empty();
}

bool same_component(const PrismaticMeshInput& input, size_t a, size_t b)
{
    return a != b && active(input, a) && active(input, b) && input.vertex_tags.at(a) == 2 &&
           input.vertex_tags.at(b) == 2 && input.vertex_component_ids.at(a) >= 0 &&
           input.vertex_component_ids.at(a) == input.vertex_component_ids.at(b) &&
           input.corr_input_vid.at(a) == input.corr_input_vid.at(b) &&
           input.corr_input_vertex.at(a) == input.corr_input_vertex.at(b);
}

bool contains(const std::array<size_t, 4>& tet, size_t v)
{
    return std::find(tet.begin(), tet.end(), v) != tet.end();
}

void rebuild_mesh(PrismaticMeshInput& input)
{
    // Compact once per collapse pass (or for an explicit standalone operation), rather
    // than rebuilding the whole domain for every accepted edge. Negative rows are holes.
    std::vector<std::array<size_t, 4>> tets;
    std::vector<int> input_cells, offset_tags;
    for (Eigen::Index i = 0; i < input.tetrahedra.rows(); ++i) {
        if (input.tetrahedra(i, 0) < 0) continue;
        std::array<size_t, 4> tet;
        for (int j = 0; j < 4; ++j) tet[j] = static_cast<size_t>(input.tetrahedra(i, j));
        tets.push_back(tet);
        input_cells.push_back(input.input_cells[i]);
        offset_tags.push_back(input.offset_tet_tags[i]);
    }
    input.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i)
        for (int j = 0; j < 4; ++j) input.tetrahedra(i, j) = static_cast<int>(tets[i][j]);
    input.input_cells = std::move(input_cells);
    input.offset_tet_tags = std::move(offset_tags);
    auto mesh = std::make_unique<TetMesh>();
    mesh->init_with_isolated_vertices(input.vertices.rows(), tets);
    input.mesh = std::move(mesh);
}

void synchronize_after_collapse(
    PrismaticMeshInput& input,
    size_t removed,
    const std::set<size_t>& affected,
    bool compact)
{
    // WMTK collapse preserves the tet IDs of retained cells and allocates no new cells.
    // Synchronize just the two endpoint stars; region tags stay in their original slots.
    for (size_t tid : affected) {
        const auto tuple = input.mesh->tuple_from_tet(tid);
        if (!tuple.is_valid(*input.mesh)) {
            input.tetrahedra.row(tid).setConstant(-1);
            continue;
        }
        const auto tet = input.mesh->oriented_tet_vids(tuple);
        for (int j = 0; j < 4; ++j) input.tetrahedra(tid, j) = static_cast<int>(tet[j]);
    }
    if (compact) rebuild_mesh(input);
    const auto cid = input.vertex_component_ids[removed];
    const auto parent = input.corr_input_vertex[removed];
    auto erase = [removed](auto& list) {
        list.erase(std::remove(list.begin(), list.end(), removed), list.end());
    };
    erase(input.offset_components.at(cid).vertices);
    erase(input.offset_vertices);
    erase(input.input_to_offset_vertices.at(parent));
    input.vertex_component_ids[removed] = -1;
    input.singular_vertex_tags[removed] = -1;
    input.offset_face_tags.clear(); // recomputed when the optimization pass is complete
}
} // namespace

bool tet_volume_above_threshold(
    const MatrixXd& vertices,
    const std::array<size_t, 4>& tet,
    double threshold)
{
    validate_threshold(threshold);
    return volume_above(vertices, tet, threshold);
}

namespace {
struct CollapsePreflight
{
    std::set<size_t> affected, modified;
    size_t background_blockers = 0;
};

std::optional<CollapsePreflight> collapse_preflight(
    const PrismaticMeshInput& input,
    size_t removed,
    size_t survivor,
    double min_tet_volume,
    bool collect_background)
{
    validate_threshold(min_tet_volume);
    if (!same_component(input, removed, survivor)) return std::nullopt;
    const auto& star = input.mesh->get_one_ring_tids_for_vertex(removed);
    bool is_edge = false;
    CollapsePreflight result;
    auto& affected = result.affected;
    affected.insert(star.begin(), star.end());
    for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(survivor)) affected.insert(tid);
    std::set<size_t> disappearing;
    auto& modified = result.modified;
    std::set<size_t> affected_vertices;
    for (size_t tid : affected) {
        auto tet = input.mesh->oriented_tet_vids(tid);
        if (!volume_above(input.vertices, tet, 0)) return std::nullopt;
        affected_vertices.insert(tet.begin(), tet.end());
        if (contains(tet, removed) && contains(tet, survivor)) {
            is_edge = true;
            disappearing.insert(tid);
            continue; // edge-incident tets are deleted, not retained as zero-volume cells
        }
        // Existing small cells may be deleted or left unchanged. Only modified shell
        // cells must meet the operation's floor; background only needs positive volume.
        if (!contains(tet, removed)) continue;
        modified.insert(tid);
        for (auto& v : tet)
            if (v == removed) v = survivor;
        const double floor = input.offset_tet_tags.at(tid) == 1 ? min_tet_volume : 0;
        if (!volume_above(input.vertices, tet, floor)) {
            if (!collect_background || input.offset_tet_tags.at(tid) == 1 ||
                input.input_cells.at(tid) == 1)
                return std::nullopt;
            ++result.background_blockers;
        }
    }
    if (!is_edge) return std::nullopt;
    // A collapse must not strand an input vertex or erase a component of the volume mesh.
    for (size_t v : affected_vertices) {
        if (v == removed) continue;
        if (v < input.fixed_background_vertices.size() && input.fixed_background_vertices[v])
            continue;
        bool retained = false;
        for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(v)) {
            if (!disappearing.count(tid)) {
                retained = true;
                break;
            }
        }
        if (!retained && v == survivor) {
            for (size_t tid : star)
                if (!disappearing.count(tid)) {
                    retained = true;
                    break;
                }
        }
        if (!retained) return std::nullopt;
    }
    auto edge = input.mesh->tuple_from_edge({removed, survivor});
    if (edge.vid(*input.mesh) != removed) edge = edge.switch_vertex(*input.mesh);
    if (!input.mesh->link_condition(edge)) return std::nullopt;
    return result;
}

bool try_collapse_offset_edge_impl(
    PrismaticMeshInput& input,
    size_t removed,
    size_t survivor,
    double min_tet_volume,
    bool compact)
{
    const auto preflight = collapse_preflight(input, removed, survivor, min_tet_volume, false);
    if (!preflight) return false;
    const auto& affected = preflight->affected;
    const auto& modified = preflight->modified;
    auto edge = input.mesh->tuple_from_edge({removed, survivor});
    if (edge.vid(*input.mesh) != removed) edge = edge.switch_vertex(*input.mesh);
    std::vector<TetMesh::Tuple> new_edges;
    if (!input.mesh->collapse_edge(edge, new_edges)) return false;
    // Check the actual result before changing external attributes. If a core connectivity
    // change ever differs from the preflight prediction, restore the original snapshot.
    for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(survivor)) {
        if (!volume_above(
                input.vertices,
                input.mesh->oriented_tet_vids(tid),
                modified.count(tid) && input.offset_tet_tags.at(tid) == 1 ? min_tet_volume : 0)) {
            rebuild_mesh(input);
            return false;
        }
    }
    synchronize_after_collapse(input, removed, affected, compact);
    return true;
}
} // namespace

std::vector<BackgroundCollapseGoal> background_blocked_collapses(
    const PrismaticMeshInput& input,
    double min_tet_volume)
{
    std::set<std::array<size_t, 2>> edges;
    for (size_t v : input.offset_vertices) {
        if (!active(input, v)) continue;
        for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(v))
            for (size_t w : input.mesh->oriented_tet_vids(tid))
                if (v < w && same_component(input, v, w)) edges.insert({v, w});
    }
    std::vector<BackgroundCollapseGoal> result;
    for (const auto& edge : edges)
        for (int direction = 0; direction < 2; ++direction) {
            const auto check = collapse_preflight(
                input,
                edge[direction],
                edge[1 - direction],
                min_tet_volume,
                true);
            if (check && check->background_blockers > 0)
                result.push_back({edge[direction], edge[1 - direction]});
        }
    return result;
}

bool try_collapse_offset_edge(
    PrismaticMeshInput& input,
    size_t removed,
    size_t survivor,
    double min_tet_volume)
{
    return try_collapse_offset_edge_impl(input, removed, survivor, min_tet_volume, true);
}

double smooth_offset_vertex(
    PrismaticMeshInput& input,
    size_t vertex,
    double min_tet_volume,
    int max_backtracks)
{
    validate_threshold(min_tet_volume);
    if (max_backtracks < 0 || max_backtracks > 60) {
        log_and_throw_error("smoothing_max_backtracks must be between 0 and 60.");
    }
    if (!active(input, vertex) || input.vertex_tags.at(vertex) != 2 ||
        input.vertex_component_ids.at(vertex) < 0 || input.singular_vertex_tags.at(vertex) != 0)
        return 0;
    const auto& component = input.offset_components.at(input.vertex_component_ids[vertex]);
    if (component.singular) return 0;
    const Vector3d start = input.vertices.row(vertex);
    const Vector3d target = component.target_position;
    if (!start.allFinite() || !target.allFinite() || start == target) return 0;
    const auto& star = input.mesh->get_one_ring_tids_for_vertex(vertex);
    for (size_t tid : star) {
        if (!volume_above(input.vertices, input.mesh->oriented_tet_vids(tid), 0)) return 0;
    }
    double alpha = 1;
    for (int backtrack = 0; backtrack <= max_backtracks; ++backtrack, alpha *= 0.5) {
        const Vector3d candidate = alpha == 1 ? target : (1 - alpha) * start + alpha * target;
        if (!candidate.allFinite() || candidate == start) return 0;
        bool valid = true;
        for (size_t tid : star) {
            if (!volume_above(
                    input.vertices,
                    input.mesh->oriented_tet_vids(tid),
                    input.offset_tet_tags.at(tid) == 1 ? min_tet_volume : 0,
                    vertex,
                    candidate)) {
                valid = false;
                break;
            }
        }
        if (!valid) continue;
        // Topology is unchanged, so all existing link relations are unchanged. A tet's
        // determinant is affine along this single-vertex segment: valid endpoints also
        // ensure no inversion along the accepted straight-line move. Shell endpoints must
        // exceed the operation's floor; background endpoints only need positive volume.
        input.vertices.row(vertex) = candidate.transpose();
        return alpha;
    }
    return 0;
}

void optimize_prismatic_mesh(PrismaticMeshInput& input, const OptimizationOptions& options)
{
    validate_threshold(options.min_tet_volume);
    if (options.iterations < 0) log_and_throw_error("iterations must be nonnegative.");
    if (options.smoothing_max_backtracks < 0 || options.smoothing_max_backtracks > 60) {
        log_and_throw_error("smoothing_max_backtracks must be between 0 and 60.");
    }
    validate_background_remeshing_options(options.background_remeshing);
    input.optimization_iterations.clear();
    input.background_remeshing_report = nullptr;
    if (options.background_remeshing.enabled) {
        input.background_remeshing_report = {
            {"enabled", true},
            {"quality_metric", "tet_mean_ratio"},
            {"quality_threshold", options.background_remeshing.quality_threshold},
            {"passes_per_iteration", options.background_remeshing.passes},
            {"max_operations_per_iteration", options.background_remeshing.max_operations},
            {"max_attempts_per_iteration", options.background_remeshing.max_attempts},
            {"iterations", nlohmann::json::array()}};
    }
    if (options.iterations == 0) return;
    if (input.vertex_component_ids.size() != static_cast<size_t>(input.vertices.rows())) {
        log_and_throw_error("Compute components and target positions before optimization.");
    }
    for (const auto& tet : input.mesh->get_tets()) {
        if (!volume_above(input.vertices, input.mesh->oriented_tet_vids(tet), 0)) {
            log_and_throw_error(
                "Initial optimization mesh tetrahedron {} has nonpositive or nonfinite volume.",
                tet.tid(*input.mesh));
        }
    }
    auto collapse_pass = [&](OptimizationIteration& stats,
                             std::set<std::array<size_t, 2>>* accepted = nullptr) {
        const size_t starting_collapses = stats.collapses;
        std::set<std::array<size_t, 2>> pending;
        auto enqueue = [&](size_t a, size_t b) {
            if (!same_component(input, a, b)) return;
            if (a > b) std::swap(a, b);
            pending.insert({a, b});
        };
        // Enumerate candidates through the offset vertices, not all background edges.
        for (size_t v : input.offset_vertices) {
            if (!active(input, v) || input.vertex_component_ids[v] < 0) continue;
            for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(v))
                for (size_t w : input.mesh->oriented_tet_vids(tid)) enqueue(v, w);
        }
        while (!pending.empty()) {
            auto endpoints = *pending.begin();
            pending.erase(pending.begin());
            if (!same_component(input, endpoints[0], endpoints[1])) continue;
            const auto& component =
                input.offset_components[input.vertex_component_ids[endpoints[0]]];
            // Prefer retaining the endpoint closest to the fixed target; try the opposite
            // direction if its geometry/topology cannot collapse safely.
            size_t survivor = endpoints[0], removed = endpoints[1];
            if (!component.singular &&
                (input.vertices.row(removed).transpose() - component.target_position)
                        .squaredNorm() <
                    (input.vertices.row(survivor).transpose() - component.target_position)
                        .squaredNorm()) {
                std::swap(survivor, removed);
            }
            ++stats.collapse_attempts;
            if (!try_collapse_offset_edge_impl(
                    input,
                    removed,
                    survivor,
                    options.min_tet_volume,
                    false)) {
                std::swap(survivor, removed);
                if (!try_collapse_offset_edge_impl(
                        input,
                        removed,
                        survivor,
                        options.min_tet_volume,
                        false))
                    continue;
            }
            ++stats.collapses;
            if (accepted) accepted->insert({removed, survivor});
            // Every candidate is reacquired by stable vertex IDs after topology changes.
            for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(survivor)) {
                const auto tet = input.mesh->oriented_tet_vids(tid);
                for (int a = 0; a < 4; ++a)
                    for (int b = a + 1; b < 4; ++b) enqueue(tet[a], tet[b]);
            }
        }
        if (stats.collapses > starting_collapses) rebuild_mesh(input);
    };
    for (int iteration = 0; iteration < options.iterations; ++iteration) {
        OptimizationIteration stats;
        collapse_pass(stats);
        stats.unlock = unlock_tau22(input, options.min_tet_volume);
        auto offset_vertices = input.offset_vertices;
        std::sort(offset_vertices.begin(), offset_vertices.end());
        for (size_t v : offset_vertices) {
            if (!active(input, v)) continue;
            if (input.vertex_tags[v] != 2 || input.vertex_component_ids[v] < 0) continue;
            if (input.singular_vertex_tags[v] == 1) {
                ++stats.singular_skipped;
                continue;
            }
            const auto& component = input.offset_components.at(input.vertex_component_ids[v]);
            if (Vector3d(input.vertices.row(v)) == component.target_position) continue;
            if (smooth_offset_vertex(
                    input,
                    v,
                    options.min_tet_volume,
                    options.smoothing_max_backtracks) > 0) {
                ++stats.smoothed_vertices;
            } else
                ++stats.smoothing_failures;
        }
        if (options.background_remeshing.enabled) {
            std::vector<BackgroundCollapseGoal> goals;
            auto report = remesh_background(
                input,
                options.min_tet_volume,
                options.background_remeshing,
                &goals);
            std::set<std::array<size_t, 2>> accepted;
            if (report["accepted_operations"].get<size_t>() > 0) collapse_pass(stats, &accepted);
            size_t rescued = 0;
            for (const auto& goal : goals) rescued += accepted.count({goal.removed, goal.survivor});
            report["additional_collapses_after_remesh"] = accepted.size();
            report["rescued_background_blocked_collapses"] = rescued;
            input.background_remeshing_report["iterations"].push_back(report);
            logger().info(
                "Background remeshing: {} 2-3 swaps, {} 3-2 swaps, {} interior moves; "
                "{} additional collapses ({} previously background-blocked); quality below {}: {} "
                "-> {}; budget exhausted={}",
                report["face_swaps_2_to_3"].get<size_t>(),
                report["edge_swaps_3_to_2"].get<size_t>(),
                report["vertex_moves"].get<size_t>(),
                accepted.size(),
                rescued,
                options.background_remeshing.quality_threshold,
                report["quality_before"]["below_threshold"].get<size_t>(),
                report["quality_after"]["below_threshold"].get<size_t>(),
                report["budget_exhausted"].get<bool>());
        }
        input.optimization_iterations.push_back(stats);
        logger().info(
            "Optimization {}/{}: {} collapses / {} candidate edges, {} smoothed, {} smoothing "
            "rejected, {} singular skipped; {} tetrahedra",
            iteration + 1,
            options.iterations,
            stats.collapses,
            stats.collapse_attempts,
            stats.smoothed_vertices,
            stats.smoothing_failures,
            stats.singular_skipped,
            input.mesh->get_tets().size());
        logger().info(
            "Unlock: {} starting tau22, {} attempted, {} unlocked, {} remaining",
            stats.unlock.candidates,
            stats.unlock.attempted,
            stats.unlock.unlocked,
            stats.unlock.remaining);
    }
    label_offset_faces(input);
}
} // namespace wmtk::components::prismatic_mesh
