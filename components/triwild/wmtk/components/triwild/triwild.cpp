#include "triwild.hpp"
#include <wmtk/utils/Preallocation.hpp>

#include <igl/Timer.h>
#include <igl/write_triangle_mesh.h>
#include <jse/jse.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include "Parameters.h"
#include "TriWildMesh.h"
#include "init_from_delaunay.hpp"

#include <triwild_spec.hpp>

namespace wmtk::components::triwild {

namespace {

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

    // Arrangement of all input meshes. Vs/Es keep the inputs as read so the winding-number
    // pass below does not have to parse every file a second time.
    MatrixXd V;
    MatrixXi F;
    MatrixXi E; // constraint edges in the arrangement
    std::vector<MatrixXd> Vs;
    std::vector<MatrixXi> Es;
    init_from_paths(input_paths, json_params["remove_duplicate_eps"], V, F, E, Vs, Es);

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
    }

    Vector2d box_min = V.colwise().minCoeff();
    Vector2d box_max = V.colwise().maxCoeff();
    params.init(box_min, box_max);

    const std::vector<std::string> tag_names = json_params["input_names"];

    TriWildMesh mesh(params, params.eps, NUM_THREADS);
    wmtk::set_preallocation_factor_from_json(mesh, json_params);
    mesh.init_mesh(V, F, E, tag_names);

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