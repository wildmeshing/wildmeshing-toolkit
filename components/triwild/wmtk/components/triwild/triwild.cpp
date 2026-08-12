#include "triwild.hpp"
#include <wmtk/utils/DriverPrologue.hpp>
#include <wmtk/utils/Preallocation.hpp>

#include <igl/Timer.h>
#include <igl/write_triangle_mesh.h>
#include <jse/jse.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SimplifySegments.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <wmtk/utils/EmbedSegments.hpp>
#include "Parameters.h"
#include "TriWildMesh.h"

#include <triwild_spec.hpp>

#include <algorithm>
#include <map>
#include <numeric>
#include <tuple>
#include <vector>

namespace wmtk::components::triwild {

namespace {

/**
 * @brief Euler characteristic of each connected component of a 2D segment network, sorted.
 *
 * The 1D counterpart of tetwild's surface version. For a graph the Euler characteristic is
 * V - E, which per connected component is 1 - (number of independent cycles): 1 for an open
 * polyline or any other tree, 0 for a single closed loop, -1 for a figure eight. Comparing
 * the sorted per-component values of the input curves against the output's tracked edges
 * therefore catches exactly the topology changes that matter here -- components merged or
 * split, loops opened or closed, whole curves lost.
 *
 * Vertices incident to no segment are ignored, mirroring the igl::remove_unreferenced call
 * in the 3D version: they are free points the arrangement still triangulates, but they
 * carry no curve topology and would otherwise swamp the result with one +1 component each.
 */
std::vector<int> compute_euler_characteristics(const MatrixXi& E)
{
    if (E.rows() == 0) {
        return {};
    }
    const int n = E.maxCoeff() + 1;
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    const auto find = [&parent](int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]]; // path halving
            x = parent[x];
        }
        return x;
    };
    for (int i = 0; i < E.rows(); ++i) {
        const int a = find(E(i, 0));
        const int b = find(E(i, 1));
        if (a != b) {
            parent[a] = b;
        }
    }

    std::vector<char> referenced(n, 0);
    for (int i = 0; i < E.rows(); ++i) {
        referenced[E(i, 0)] = 1;
        referenced[E(i, 1)] = 1;
    }
    std::map<int, int> n_verts, n_edges;
    for (int v = 0; v < n; ++v) {
        if (referenced[v]) {
            ++n_verts[find(v)];
        }
    }
    for (int i = 0; i < E.rows(); ++i) {
        ++n_edges[find(E(i, 0))];
    }

    std::vector<int> ecs;
    ecs.reserve(n_verts.size());
    for (const auto& [root, nv] : n_verts) {
        ecs.push_back(nv - n_edges[root]);
    }
    std::sort(ecs.begin(), ecs.end());
    return ecs;
}

/**
 * @brief Largest distance from any point of the segments (V, E) to the geometry that
 * `target` was built over, or -1 when there is nothing to sample.
 *
 * Samples each segment uniformly with the budget spread by length: the input is a segment
 * soup, so there is no area to sample, and there is no reason to sample a long segment as
 * coarsely as a short one. Deterministic, unlike the 3D version's
 * igl::random_points_on_mesh -- a diagnostic that changes between runs is hard to act on.
 *
 * A max over samples converges slowly, so the budget is deliberately large (tetwild raised
 * its own from 10k to 100k after finding a 21x under-estimate). This is opt-in diagnostic
 * code; a stable answer is worth the samples.
 */
struct Deviation
{
    double max = -1;
    /// Mean over the same samples. Because the budget is already spread by length, this is
    /// the mean along the curve rather than the mean over segments, so a dense patch of short
    /// segments does not outvote a long one. The max alone hides what a change did to the
    /// bulk of the curve: it is a single worst sample, and one stubborn corner pins it while
    /// everything else moves.
    double mean = -1;
};

Deviation
deviation_stats(const MatrixXd& V, const MatrixXi& E, const SampleEnvelope& target, int n_samples)
{
    double total_length = 0;
    for (int i = 0; i < E.rows(); ++i) {
        total_length += (V.row(E(i, 1)) - V.row(E(i, 0))).norm();
    }
    if (E.rows() == 0 || total_length <= 0) {
        return {};
    }

    double sq_max = -1;
    double sum = 0;
    long long count = 0;
    Vector2d projection;
    for (int i = 0; i < E.rows(); ++i) {
        const Vector2d a = V.row(E(i, 0));
        const Vector2d b = V.row(E(i, 1));
        const double len = (b - a).norm();
        const int n = std::max(1, int(n_samples * len / total_length));
        for (int s = 0; s <= n; ++s) {
            const Vector2d p = a + (b - a) * (double(s) / double(n));
            // nearest_point returns the SQUARED distance.
            const double sq = target.nearest_point(p, projection);
            sq_max = std::max(sq_max, sq);
            sum += std::sqrt(sq);
            ++count;
        }
    }
    if (sq_max < 0 || count == 0) {
        return {};
    }
    return {std::sqrt(sq_max), sum / double(count)};
}

/**
 * @brief Deviation of the tracked VERTICES alone, ignoring the chords between them.
 *
 * Separates the two things the edge-sampled figure mixes together. Smoothing places
 * vertices, so a vertex's distance to the input is what the envelope penalty actually
 * controls. The interior of a tracked edge is a chord across whatever the input does between
 * its endpoints, so it carries the discretization error that collapse left behind -- no
 * weight on the penalty can pull a chord onto a curve whose intermediate vertex was removed.
 *
 * Reported alongside the edge figure so a saturating edge deviation can be attributed to one
 * or the other rather than guessed at.
 */
Deviation vertex_deviation(const MatrixXd& V, const MatrixXi& E, const SampleEnvelope& target)
{
    std::vector<bool> used(V.rows(), false);
    for (int i = 0; i < E.rows(); ++i) {
        used[E(i, 0)] = true;
        used[E(i, 1)] = true;
    }

    double sq_max = -1;
    double sum = 0;
    long long count = 0;
    Vector2d projection;
    for (int i = 0; i < V.rows(); ++i) {
        if (!used[i]) {
            continue;
        }
        const double sq = target.nearest_point(Vector2d(V.row(i)), projection);
        sq_max = std::max(sq_max, sq);
        sum += std::sqrt(sq);
        ++count;
    }
    if (count == 0 || sq_max < 0) {
        return {};
    }
    return {std::sqrt(sq_max), sum / double(count)};
}

/**
 * @brief The output's tracked (input-carrying) edges, as a standalone (V, E) pair.
 */
void tracked_edges(const TriWildMesh& mesh, MatrixXd& V, MatrixXi& E)
{
    V.resize(mesh.vert_capacity(), 2);
    for (size_t i = 0; i < mesh.vert_capacity(); ++i) {
        V.row(i) = mesh.m_vertex_attribute[i].m_posf;
    }
    const auto tracked =
        mesh.get_edges_by_condition([](const auto& e) { return e.m_is_surface_fs; });
    E.resize(tracked.size(), 2);
    for (size_t i = 0; i < tracked.size(); ++i) {
        E(i, 0) = int(tracked[i][0]);
        E(i, 1) = int(tracked[i][1]);
    }
}

/**
 * @brief Build a 2D envelope over the segments (V, E). eps is irrelevant here: only
 * nearest_point() is used, and that ignores the thickness.
 */
std::shared_ptr<SampleEnvelope> envelope_over(const MatrixXd& V, const MatrixXi& E)
{
    std::vector<Vector2d> v(V.rows());
    for (int i = 0; i < V.rows(); ++i) {
        v[i] = V.row(i);
    }
    std::vector<Vector2i> e(E.rows());
    for (int i = 0; i < E.rows(); ++i) {
        e[i] = E.row(i);
    }
    auto env = std::make_shared<SampleEnvelope>();
    env->init(v, e, 0);
    return env;
}

} // namespace

void triwild(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    const std::filesystem::path root = utils::verify_and_setup_logger(
        json_params,
        jse::embed::wmtk_triwild_spec::triwild_spec::spec(),
        false);
    const std::vector<std::string> input_paths = utils::resolve_input_paths(json_params, root);

    triwild::Parameters params(json_params);

    std::string output_path = json_params["output"];
    int NUM_THREADS = json_params["num_threads"];
    int max_its = json_params["max_iterations"];

    const std::string filter_option = json_params["filter"];

    // Read the inputs. Vs/Es keep them per input, as read, so the winding-number pass below
    // does not have to parse every file a second time; V_in/E_in is their union, which is
    // what gets simplified and arranged.
    MatrixXd V_in;
    MatrixXi E_in;
    std::vector<MatrixXd> Vs;
    std::vector<MatrixXi> Es;
    wmtk::utils::read_input_curves(
        input_paths,
        json_params["remove_duplicate_eps"],
        V_in,
        E_in,
        Vs,
        Es);

    // Informational input-topology report; gated behind DEBUG_euler because it is only
    // meaningful next to the matching computations later in the run.
    const bool debug_euler = json_params["DEBUG_euler"];
    std::vector<int> ecs_input;
    if (debug_euler) {
        ecs_input = compute_euler_characteristics(E_in);
        logger().info("Euler characteristic, input curves: {}", ecs_input);
    }

    // The bounding box comes from the input curves, as in tetwild -- eps has to be known
    // before the arrangement runs, because the simplification happens first. (It used to be
    // taken from the arrangement output, which includes the background grid and is about
    // 13% larger.)
    params.init(V_in.colwise().minCoeff(), V_in.colwise().maxCoeff());

    // The whole eps around the original input, as in tetwild. This used to be params.eps / 2
    // on the grounds that "the arrangement gets the other half, which is what leaves room for
    // the optimizer to move" -- but reserving that room is what simplify_envelope_ratio does,
    // immediately below, so the two compounded and the simplification ran at eps/4 while the
    // optimizer was held to half the tolerance its result is judged against.
    const double envelope_eps = params.eps;
    // The simplification and the triangulation used to share one envelope object at the same
    // eps, which leaves the optimizer no room: a simplification free to place a vertex right
    // at the limit hands the optimizer a mesh where almost every move is already outside.
    // Simplifying inside a fraction of the envelope reserves the rest as headroom. Measured
    // in 3D, where sharing one envelope put a third of the simplified vertices outside before
    // the tet phase began and had ~94% of smoothing attempts vetoed.
    const bool simplify_use_link_condition = json_params["simplify_use_link_condition"];
    const double simplify_envelope_ratio = json_params["simplify_envelope_ratio"];
    const double simplify_eps = envelope_eps * simplify_envelope_ratio;
    logger().info(
        "envelope eps: simplification {:.6} ({:.0f}% of triangulation {:.6})",
        simplify_eps,
        100 * simplify_envelope_ratio,
        envelope_eps);
    const bool simplify_use_sample_envelope = json_params["simplify_use_sample_envelope"];
    // The only ratio that is wrong independently of how the envelope is implemented. Above 1
    // the simplification is allowed to move geometry further than the triangulation's own
    // envelope permits, so the optimizer starts from a mesh that is already outside it and
    // every operation near those curves is vetoed.
    //
    // There used to be a warning at 0.5 here. That number was the sampled backend's internal
    // compensation showing through -- its edge test accepts a segment only within eps/2,
    // because its sampling guarantees the other eps/2 -- and both backends guarantee the same
    // thing to their caller: everything they accept is within eps. A caller reasoning about
    // eps/2 is reasoning about an implementation detail it should not be able to see.
    if (simplify_envelope_ratio > 1.0) {
        logger().warn(
            "simplify_envelope_ratio {} exceeds 1; the simplification may move curves further "
            "than the triangulation's envelope allows, leaving the mesh outside it at init.",
            simplify_envelope_ratio);
    }

    MatrixXd V_simp = V_in;
    MatrixXi E_simp = E_in;
    if (json_params["skip_simplify"]) {
        logger().info("skip simplification");
    } else {
        SampleEnvelope simplify_envelope(!simplify_use_sample_envelope);
        simplify_envelope.init(V_in, E_in, simplify_eps);
        const size_t removed = wmtk::utils::simplify_segments(
            V_simp,
            E_simp,
            simplify_envelope,
            simplify_use_link_condition);
        logger().info(
            "input simplification: #V {} -> {}, #E {} -> {} ({} vertices removed). Link "
            "condition {}.",
            V_in.rows(),
            V_simp.rows(),
            E_in.rows(),
            E_simp.rows(),
            removed,
            simplify_use_link_condition ? "on" : "off");
    }

    // The simplification is the one stage whose topology is genuinely optional: with
    // simplify_use_link_condition off it may merge junctions and separate curves. Compare
    // here, while the curves are still the same kind of object.
    std::vector<int> ecs_simplified;
    if (debug_euler) {
        ecs_simplified = compute_euler_characteristics(E_simp);
        if (ecs_simplified != ecs_input) {
            logger().warn(
                "Euler characteristic, after simplification: {} -- changed from the input "
                "{}. Expected when simplify_use_link_condition is off, which allows "
                "junctions and separate curves to merge.",
                ecs_simplified,
                ecs_input);
        } else {
            logger().info("Euler characteristic, after simplification: unchanged.");
        }
    }

    // Exact arrangement of the (simplified) segment network.
    MatrixXd V;
    std::vector<Vector2r> V_rational; // the same vertices, exact
    MatrixXi F;
    MatrixXi E; // constraint edges in the arrangement
    wmtk::utils::embed_segments(V_simp, E_simp, V, V_rational, F, E);

    // The arrangement is the baseline for everything after it. It is EXPECTED to differ
    // from the input: resolving a crossing inserts a vertex shared by both curves, which
    // merges two components into one and adds a cycle. That is the arrangement doing its
    // job, not a defect -- so this is reported, never warned about.
    std::vector<int> ecs_arrangement;
    if (debug_euler) {
        ecs_arrangement = compute_euler_characteristics(E);
        logger().info(
            "Euler characteristic, arrangement constraints: {} ({} constrained edges)",
            ecs_arrangement,
            E.rows());
    }

    if (params.debug_output) {
        MatrixXd V3(V.rows(), 3);
        V3.setZero();
        V3.block(0, 0, V.rows(), 2) = V;
        igl::write_triangle_mesh(output_path + "_initial_delaunay.obj", V3, F);

        // write edges
        std::ofstream edge_out(output_path + "_initial_edges.obj");
        for (int i = 0; i < E.rows(); i++) {
            edge_out << "v " << V(E(i, 0), 0) << " " << V(E(i, 0), 1) << " 0\n";
            edge_out << "v " << V(E(i, 1), 0) << " " << V(E(i, 1), 1) << " 0\n";
            edge_out << "l " << 2 * i + 1 << " " << 2 * i + 2 << "\n";
        }
        edge_out.close();

        // and the simplified input, the counterpart of tetwild's _simplified_input.obj
        std::ofstream simp_out(output_path + "_simplified_input.obj");
        for (int i = 0; i < V_simp.rows(); i++) {
            simp_out << "v " << V_simp(i, 0) << " " << V_simp(i, 1) << " 0\n";
        }
        for (int i = 0; i < E_simp.rows(); i++) {
            simp_out << "l " << E_simp(i, 0) + 1 << " " << E_simp(i, 1) + 1 << "\n";
        }
        simp_out.close();
    }

    const std::vector<std::string> tag_names = json_params["input_names"];

    TriWildMesh mesh(params, envelope_eps, NUM_THREADS);
    wmtk::set_preallocation_factor_from_json(mesh, json_params);
    mesh.init_mesh(V, V_rational, F, E, tag_names, V_in, E_in);

    // After init_mesh, which is what builds the envelope, and after the simplification, which
    // uses its own object -- so this only disables the checks the optimizer makes.
    if (json_params["DEBUG_disable_envelope"]) {
        logger().warn(
            "DEBUG_disable_envelope: envelope checks are OFF for the triangulation. "
            "The output has no containment guarantee and is for diagnosis only.");
        mesh.m_envelope->disabled = true;
    }

    if (params.debug_output) {
        mesh.write_vtu(output_path + "_initial");
    }

    /////////mesh improvement
    mesh.mesh_improvement(max_its);
    mesh.consolidate_mesh();

    bool all_rounded = true;
    for (const auto& v : mesh.get_vertices()) {
        if (!mesh.m_vertex_attribute[v.vid(mesh)].m_is_rounded) {
            all_rounded = false;
            break;
        }
    }
    if (all_rounded) {
        logger().info("All vertices are rounded");
    } else {
        logger().error("Not all vertices rounded!");
    }

    // Flood-fill part ids and per-input winding-number tags. Both are needed to filter the
    // outside region (filter != "none"); the tags additionally drive the MSH groups. When
    // nothing needs them, skip_winding_number lets a caller opt out -- at the cost of the
    // output groups, since without the tags every face lands in the untagged group.
    const bool skip_winding = json_params["skip_winding_number"] && filter_option == "none";
    if (json_params["skip_winding_number"] && filter_option != "none") {
        logger().warn(
            "skip_winding_number is set but filter='{}' requires the winding number; "
            "computing it anyway.",
            filter_option);
    }
    if (!skip_winding) {
        int num_parts = mesh.flood_fill();
        logger().info("flood fill parts {}", num_parts);
        mesh.compute_winding_numbers(Vs, Es);
    } else {
        logger().info(
            "Skipping winding-number and flood-fill computation (skip_winding_number). The "
            "output groups will be empty.");
    }

    if (filter_option == "input") {
        mesh.filter_with_input_winding_number();
        mesh.consolidate_mesh();
    } else if (filter_option == "flood") {
        mesh.filter_with_flood_fill();
        mesh.consolidate_mesh();
    } else if (filter_option != "none") {
        logger().error("Unknown filter option '{}'. No filtering performed.", filter_option);
    }

    if (mesh.tri_capacity() == 0) {
        log_and_throw_error("Empty Output after Filter!");
    }

    // double time = timer.getElapsedTime();
    // logger().info("total time {:.4}s", time);

    // Surface deviation, both directions. The 2D counterpart of tetwild's DEBUG_hausdorff,
    // including the correction from #967:
    //
    //   containment d(output -> input) is the envelope invariant. Every point of the output's
    //       tracked edges stays within eps of the input; this is what the optimizer maintains
    //       and the only direction worth comparing against eps.
    //   coverage    d(input -> output) is NOT promised by anything. The simplification is
    //       allowed to remove detail and the arrangement may drop segments, so this grows
    //       while containment is untouched. Reported as a diagnostic only.
    //
    // This check used to compute coverage, call it "Hausdorff distance", and warn that it was
    // "larger than the envelope" -- exactly the mistake #967 fixed in 3D, where a legitimate
    // result read 50x eps on containment that was actually 0.34x. On the 2D sweep it made ~60%
    // of successful models look like envelope violations.
    Deviation containment;
    Deviation coverage;
    Deviation containment_v;
    if (json_params["DEBUG_hausdorff"]) {
        const int n_samples = 100000;
        MatrixXd V_track;
        MatrixXi E_track;
        tracked_edges(mesh, V_track, E_track);
        if (E_track.rows() == 0) {
            logger().warn("Hausdorff check: the output has no tracked edges.");
        } else {
            const auto env_in = envelope_over(V_in, E_in);
            const auto env_out = envelope_over(V_track, E_track);
            containment = deviation_stats(V_track, E_track, *env_in, n_samples);
            coverage = deviation_stats(V_in, E_in, *env_out, n_samples);
            containment_v = vertex_deviation(V_track, E_track, *env_in);

            logger().info(
                "curve deviation: containment over tracked VERTICES only max = {:.4}, mean = "
                "{:.4}",
                containment_v.max,
                containment_v.mean);

            logger().info(
                "curve deviation: containment d(output->input) max = {:.4}, mean = {:.4} | "
                "envelope = {:.4}",
                containment.max,
                containment.mean,
                params.eps);
            if (containment.max > params.eps) {
                logger().warn(
                    "Output is outside the envelope; the containment invariant was "
                    "violated.");
            } else {
                logger().info("Output is inside the envelope (as expected).");
            }
            // No comparison against eps: the pipeline does not promise this direction.
            logger().info(
                "curve deviation: coverage d(input->output) max = {:.4}, mean = {:.4} "
                "(diagnostic; large means the output no longer covers part of the input)",
                coverage.max,
                coverage.mean);
        }
    }

    // Output topology, compared against the ARRANGEMENT, not against the input. The
    // arrangement is where a legitimate topology change happens -- resolving a crossing
    // merges two components and adds a cycle -- so comparing to the input would warn on
    // every self-intersecting drawing. Everything after the arrangement (split, collapse,
    // swap) is supposed to leave the tracked network's topology alone, and a collapse is
    // the one that could quietly not: the envelope only bounds output-inside-input, so it
    // cannot see a curve eroding along itself.
    std::vector<int> ecs_output;
    if (debug_euler) {
        const auto tracked =
            mesh.get_edges_by_condition([](const auto& e) { return e.m_is_surface_fs; });
        MatrixXi E_out(tracked.size(), 2);
        for (size_t i = 0; i < tracked.size(); ++i) {
            E_out(i, 0) = int(tracked[i][0]);
            E_out(i, 1) = int(tracked[i][1]);
        }
        ecs_output = compute_euler_characteristics(E_out);
        logger().info(
            "Euler characteristic, output tracked edges: {} ({} edges)",
            ecs_output,
            E_out.rows());
        if (ecs_output != ecs_arrangement) {
            logger().warn(
                "Euler characteristic changed across the optimization: arrangement {} -> "
                "output {}. The operations altered the tracked curve network's topology; "
                "only the arrangement is entitled to do that.",
                ecs_arrangement,
                ecs_output);
        } else {
            logger().info("Euler characteristic is unchanged across the optimization.");
        }
    }

    // The feature-point invariant, measured on the finished mesh. Opt-in: the cost scales
    // with the input's feature count, which on a dense curve network runs to hundreds of
    // thousands, and it tells you nothing the run depends on.
    double feat_worst_ratio = 0;
    size_t feat_kept = 0, feat_total = 0;
    if (json_params["DEBUG_feature_retention"]) {
        std::tie(feat_kept, feat_total) = mesh.feature_retention(&feat_worst_ratio);
    }
    if (feat_total > 0) {
        if (feat_kept == feat_total) {
            logger().info("feature points retained: {}/{}", feat_kept, feat_total);
        } else {
            logger().warn(
                "feature points retained: {}/{} -- {} polyline endpoints or junctions are no "
                "longer represented within eps; the worst is {:.2f} x eps from the nearest "
                "vertex",
                feat_kept,
                feat_total,
                feat_total - feat_kept,
                feat_worst_ratio);
        }
    }

    /////////output
    auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);

    const std::string report_file = json_params["report"];
    if (!report_file.empty()) {
        std::ofstream fout(report_file);
        nlohmann::json report;
        report["#t"] = mesh.tri_capacity();
        report["#v"] = mesh.vert_capacity();
        report["max_energy"] = max_energy;
        report["avg_energy"] = avg_energy;
        report["eps"] = params.eps;
        report["threads"] = NUM_THREADS;
        report["#iterations"] = mesh.m_iterations_used;
        // report["time"] = time;
        // "hausdorff" keeps its name and now holds containment, the invariant; "coverage" is
        // the other direction. Same convention as the tetwild report.
        report["hausdorff"] = containment.max;
        report["hausdorff_mean"] = containment.mean;
        report["hausdorff_vertex"] = containment_v.max;
        report["hausdorff_vertex_mean"] = containment_v.mean;
        report["coverage"] = coverage.max;
        report["coverage_mean"] = coverage.mean;
        if (json_params["DEBUG_feature_retention"]) {
            report["features_retained"] = feat_kept;
            report["features_total"] = feat_total;
            report["features_worst_ratio"] = feat_worst_ratio;
        }
        report["all_rounded"] = all_rounded;
        // report["insertion_and_preprocessing"] = insertion_time;
        fout << std::setw(4) << report;
        fout.close();
    }

    // check metrics
    if (json_params["throw_on_fail"]) {
        if (!all_rounded) {
            log_and_throw_error("Not all vertices rounded!");
        }
        if (max_energy > params.stop_energy) {
            log_and_throw_error("Max energy is too large.");
        }
        // Regression guard for the sizing-refinement stall: an input that normally converges
        // in a handful of iterations and suddenly needs the whole budget means the
        // stuck-refine trigger stopped firing when it should. That is a silent slowdown --
        // the mesh still reaches stop_energy, so no energy or envelope assertion catches it.
        const int max_expected_its = json_params["max_expected_iterations"];
        if (max_expected_its > 0 && mesh.m_iterations_used > max_expected_its) {
            log_and_throw_error(
                "Converged, but needed {} iterations against an expected maximum of {}.",
                mesh.m_iterations_used,
                max_expected_its);
        }
        // Feature-point retention is deliberately NOT asserted here, because "every anchor
        // keeps a vertex within eps" is not actually an invariant of the pipeline.
        //
        // Two anchors closer than eps are allowed to merge: the survivor covers both, which
        // is correct and is what keeps a mesh with sub-eps features from deadlocking. But the
        // survivor carries only ONE feature id, so the anchor it did not keep stops
        // constraining it, and it may then move a further eps away from that one.
        //
        // Those merges CASCADE. The survivor can merge again with a third anchor, and again
        // with a fourth, each step re-anchoring the constraint on whichever id it happens to
        // carry and letting the earliest anchors slip another eps. A chain of k merges walks
        // the vertex O(k * eps) from where it started, so no fixed multiple of eps is a
        // sound bound -- it depends on how many sub-eps features the input crowds together.
        // Measured on the 2D sweep the worst case is 2.22 x eps, but that is an observation,
        // not a guarantee, and throwing on any particular multiple would be dressing up a
        // guess as an invariant.
        //
        // Making it exact needs per-vertex feature SETS, so a survivor stays constrained by
        // every anchor it covers rather than the last id assigned to it. Until then this is
        // reported and not enforced: the warning above, plus features_retained and
        // features_worst_ratio in the report.
    }

    if (json_params["write_vtu"]) {
        mesh.write_vtu(output_path);
    }
    mesh.write_msh_groups(output_path + ".msh");

    logger().info("======= finish =========");
}

} // namespace wmtk::components::triwild