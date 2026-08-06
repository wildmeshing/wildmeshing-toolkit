#include "triwild.hpp"
#include <wmtk/utils/Preallocation.hpp>

#include <igl/Timer.h>
#include <igl/write_triangle_mesh.h>
#include <jse/jse.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SimplifySegments.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include "Parameters.h"
#include "TriWildMesh.h"
#include "init_from_delaunay.hpp"

#include <triwild_spec.hpp>

#include <algorithm>
#include <map>
#include <numeric>
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
 * @brief One-sided Hausdorff distance from the input segments to the output's tracked
 * edges: the largest distance any point of the input has to travel to reach the output.
 *
 * The 2D counterpart of tetwild's DEBUG_hausdorff check, which samples the input surface
 * and measures against an envelope built over the output surface. Here the samples are
 * spread along each input segment proportionally to its length (the input is a segment
 * soup, so there is no area to sample uniformly).
 */
double hausdorff_to_output(
    const TriWildMesh& mesh,
    const std::vector<MatrixXd>& Vs,
    const std::vector<MatrixXi>& Es)
{
    // Envelope over the output's tracked edges.
    std::vector<Vector2d> v_out;
    std::vector<Vector2i> e_out;
    v_out.reserve(mesh.vert_capacity());
    for (size_t i = 0; i < mesh.vert_capacity(); ++i) {
        v_out.push_back(mesh.m_vertex_attribute[i].m_posf);
    }
    for (const auto& vids :
         mesh.get_edges_by_condition([](const auto& e) { return e.m_is_surface_fs; })) {
        e_out.emplace_back(int(vids[0]), int(vids[1]));
    }
    if (e_out.empty()) {
        logger().warn("Hausdorff check: the output has no tracked edges.");
        return -1;
    }
    SampleEnvelope env;
    env.init(v_out, e_out, 0);

    // Total input length, so the sample budget can be spread by length.
    constexpr int n_samples = 10000;
    double total_length = 0;
    for (size_t k = 0; k < Es.size(); ++k) {
        for (int i = 0; i < Es[k].rows(); ++i) {
            total_length += (Vs[k].row(Es[k](i, 1)) - Vs[k].row(Es[k](i, 0))).norm();
        }
    }
    if (total_length <= 0) {
        return -1;
    }

    double sq_max = -1;
    Vector2d projection;
    for (size_t k = 0; k < Es.size(); ++k) {
        for (int i = 0; i < Es[k].rows(); ++i) {
            const Vector2d a = Vs[k].row(Es[k](i, 0));
            const Vector2d b = Vs[k].row(Es[k](i, 1));
            const double len = (b - a).norm();
            const int n = std::max(1, int(n_samples * len / total_length));
            for (int s = 0; s <= n; ++s) {
                const Vector2d p = a + (b - a) * (double(s) / double(n));
                sq_max = std::max(sq_max, env.nearest_point(p, projection));
            }
        }
    }
    return sq_max < 0 ? -1 : std::sqrt(sq_max);
}

} // namespace

void triwild(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        const auto spec = jse::embed::wmtk_triwild_spec::triwild_spec::spec();
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
    }
    const std::filesystem::path root = json_params["input_dir"];

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            log_file_name = resolve_path(root, log_file_name).string();
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    std::vector<std::string> input_paths = json_params["input"];
    for (std::string& p : input_paths) {
        p = resolve_path(root, p).string();
    }

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
    read_input_curves(input_paths, json_params["remove_duplicate_eps"], V_in, E_in, Vs, Es);

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

    // Half of eps around the original input, as in tetwild -- the arrangement gets the other
    // half, which is what leaves room for the optimizer to move.
    const double envelope_eps = params.eps / 2;
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

    MatrixXd V_simp = V_in;
    MatrixXi E_simp = E_in;
    if (json_params["skip_simplify"]) {
        logger().info("skip simplification");
    } else {
        SampleEnvelope simplify_envelope;
        {
            std::vector<Vector2d> v(V_in.rows());
            for (int i = 0; i < V_in.rows(); ++i) {
                v[i] = V_in.row(i);
            }
            std::vector<Vector2i> e(E_in.rows());
            for (int i = 0; i < E_in.rows(); ++i) {
                e[i] = E_in.row(i);
            }
            simplify_envelope.init(v, e, simplify_eps);
        }
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
    init_from_delaunay_box_mesh(V_simp, E_simp, V, V_rational, F, E);

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

    // Sanity check: how far the input actually is from the output. The 2D counterpart of
    // tetwild's DEBUG_hausdorff -- sample the input segments and measure each sample
    // against an envelope built over the output's tracked (surface) edges.
    double hausdorff_distance = -1;
    if (json_params["DEBUG_hausdorff"]) {
        hausdorff_distance = hausdorff_to_output(mesh, Vs, Es);
        logger().info(
            "Hausdorff distance = {:.4} | Envelope = {:.4}",
            hausdorff_distance,
            params.eps);
        if (hausdorff_distance > params.eps) {
            logger().warn("Hausdorff distance is larger than the envelope!");
        } else {
            logger().info("Hausdorff distance is smaller than envelope (as expected).");
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
        // report["time"] = time;
        report["hausdorff"] = hausdorff_distance;
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
    }

    if (json_params["write_vtu"]) {
        mesh.write_vtu(output_path);
    }
    mesh.write_msh_groups(output_path + ".msh");

    logger().info("======= finish =========");
}

} // namespace wmtk::components::triwild