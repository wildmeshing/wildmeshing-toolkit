#include <CLI/CLI.hpp>

#include <igl/Timer.h>

#include <stdlib.h>
#include <chrono>
#include <cstdlib>
#include <iostream>

#include <jse/jse.h>
#include <nlohmann/json.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include "ShortestEdgeCollapse.h"

#include <shortest_edge_collapse_spec.hpp>

namespace wmtk::components::shortest_edge_collapse {

void run_shortest_collapse(
    std::string input,
    int target,
    std::string output,
    ShortestEdgeCollapse& m)
{
    igl::Timer timer;
    logger().info("target #V = {}", target);
    if (!m.check_mesh_connectivity_validity()) {
        log_and_throw_error("mesh is invalid");
    }
    timer.start();
    m.collapse_shortest(target);
    timer.stop();
    auto stop = std::chrono::high_resolution_clock::now();
    logger().info("runtime {:.4}s", timer.getElapsedTimeInSec());
    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    // m.write_vtu(std::filesystem::path(output).stem().string());
    logger().info("After collapse: #V = {}, #F = {}", m.vert_capacity(), m.tri_capacity());
}

void shortest_edge_collapse(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        const auto spec =
            jse::embed::wmtk_shortest_edge_collapse_spec::shortest_edge_collapse_spec::spec();
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
    }
    const std::filesystem::path root = json_params["input_dir"];

    const std::string path = resolve_path(root, json_params["input"]).string();
    const std::string output = json_params["output"];
    const double env_rel = json_params["eps_rel"];
    const bool use_sample_envelope = json_params["use_sample_envelope"];
    const double target_pec = json_params["target_rel"];
    const int num_threads = json_params["num_threads"];

    MatrixXd V;
    MatrixXi F;
    io::read_triangle_mesh(path, V, F, -1, -1);
    logger().info("Input: #V = {}, #F = {}", V.rows(), F.rows());


    std::vector<Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            tri[i][j] = (size_t)F(i, j);
        }
    }

    const MatrixXd box_min = V.colwise().minCoeff();
    const MatrixXd box_max = V.colwise().maxCoeff();
    const double diag = (box_max - box_min).norm();

    const double envelope_size = env_rel * diag;
    VectorXi dummy;
    std::vector<size_t> modified_v;

    ShortestEdgeCollapse m(v, num_threads, !use_sample_envelope);
    m.create_mesh(v.size(), tri, modified_v, envelope_size);
    if (!m.check_mesh_connectivity_validity()) {
        log_and_throw_error("Mesh connectivity is invalid!");
    }
    logger().info("collapsing mesh {}", path);
    int target_verts = v.size() * target_pec;
    run_shortest_collapse(path, target_verts, output, m);

    if (json_params["throw_on_fail"]) {
        const size_t nv = m.get_vertices().size();
        if (nv > target_verts) {
            log_and_throw_error("Target not reached: #V = {}, target was {}", target_verts, nv);
        }
    }

    logger().info("===== finished =====");
}

} // namespace wmtk::components::shortest_edge_collapse