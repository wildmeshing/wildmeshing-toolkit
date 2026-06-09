#include "triwild.hpp"

#include <igl/Timer.h>
#include <igl/write_triangle_mesh.h>
#include <jse/jse.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include "Parameters.h"
#include "TriWildMesh.h"
#include "init_from_delaunay.hpp"
#include "triwild_spec.hpp"

namespace wmtk::components::triwild {

void triwild(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, triwild_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, triwild_spec);
    }
    const std::filesystem::path root =
        json_params.contains("json_input_file") ? json_params["json_input_file"] : "";

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

    triwild::Parameters params;

    std::string output_path = json_params["output"];
    bool skip_simplify = json_params["skip_simplify"];
    int NUM_THREADS = json_params["num_threads"];
    int max_its = json_params["max_iterations"];
    std::string filter_option = json_params["filter"];

    params.epsr = json_params["eps_rel"];
    params.lr = json_params["length_rel"];
    params.stop_energy = json_params["stop_energy"];

    params.preserve_topology = json_params["preserve_topology"];

    params.debug_output = json_params["DEBUG_output"];
    params.perform_sanity_checks = json_params["DEBUG_sanity_checks"];

    // CDT on all input meshes
    MatrixXd V;
    MatrixXi F;
    MatrixXi E; // constraint edges in CDT
    init_from_paths(input_paths, V, F, E);

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

    TriWildMesh mesh(params, params.eps, NUM_THREADS);
    mesh.init_mesh(V, F, E);

    if (params.debug_output) {
        mesh.write_vtu(output_path + "_initial");
    }

    /////////mesh improvement
    mesh.mesh_improvement(max_its);

    // bool all_rounded = true;
    // for (const auto& v : mesh_new.get_vertices()) {
    //     if (!mesh_new.m_vertex_attribute[v.vid(mesh_new)].m_is_rounded) {
    //         all_rounded = false;
    //         break;
    //     }
    // }
    // if (all_rounded) {
    //     wmtk::logger().info("All vertices are rounded");
    // } else {
    //     wmtk::logger().error("Not all vertices rounded!");
    // }

    // // apply input winding number
    // mesh_new.compute_winding_number(verts, tris);
    // // apply tracked surface winding number
    // mesh_new.compute_winding_number();
    // // apply flood fill
    // {
    //     int num_parts = mesh_new.flood_fill();
    //     logger().info("flood fill parts {}", num_parts);
    // }
    // // compute per-input winding number
    // mesh_new.compute_winding_numbers(input_paths);

    // // ////winding number
    // if (filter_option == "input") {
    //     mesh_new.filter_with_input_surface_winding_number();
    // } else if (filter_option == "tracked") {
    //     mesh_new.filter_with_tracked_surface_winding_number();
    // } else if (filter_option == "flood") {
    //     mesh_new.filter_with_flood_fill();
    // } else if (filter_option != "none") {
    //     logger().error("Unknown filter option '{}'. No filtering performed.", filter_option);
    // }
    // mesh_new.consolidate_mesh();

    // double time = timer.getElapsedTime();
    // logger().info("total time {:.4}s", time);
    // if (mesh_new.tet_size() == 0) {
    //     log_and_throw_error("Empty Output after Filter!");
    // }

    // /////////output
    // auto [max_energy, avg_energy] = mesh_new.get_max_avg_energy();
    // wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);

    // const std::string report_file = json_params["report"];
    // if (!report_file.empty()) {
    //     std::ofstream fout(report_file);
    //     nlohmann::json report;
    //     report["#t"] = mesh_new.tet_size();
    //     report["#v"] = mesh_new.vertex_size();
    //     report["max_energy"] = max_energy;
    //     report["avg_energy"] = avg_energy;
    //     report["eps"] = params.eps;
    //     report["threads"] = NUM_THREADS;
    //     report["time"] = time;
    //     report["all_rounded"] = all_rounded;
    //     report["insertion_and_preprocessing"] = insertion_time;
    //     fout << std::setw(4) << report;
    //     fout.close();
    // }

    // // check metrics
    // if (json_params["throw_on_fail"]) {
    //     if (!all_rounded) {
    //         log_and_throw_error("Not all vertices rounded!");
    //     }
    //     if (max_energy > params.stop_energy) {
    //         log_and_throw_error("Max energy is too large.");
    //     }
    // }

    // mesh_new.write_vtu(output_path);
    // mesh_new.write_msh_groups(output_path + "_final.msh");

    logger().info("======= finish =========");
}

} // namespace wmtk::components::triwild