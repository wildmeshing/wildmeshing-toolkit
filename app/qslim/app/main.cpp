#include <qslim/QSLIM.h>

#include <wmtk/utils/ManifoldUtils.hpp>

#include <CLI/CLI.hpp>

#include <igl/Timer.h>
#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <wmtk/utils/Reader.hpp>

#include <jse/jse.h>
#include <stdlib.h>
#include <chrono>
#include <cstdlib>
#include <iostream>

using namespace wmtk;
using namespace app::qslim;
using namespace std::chrono;

#include "qslim_spec.hpp"

void run_qslim_collapse(std::string input, int target, std::string output, QSLIM& m)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target number of verts: {}", target);
    assert(m.check_mesh_connectivity_validity());
    wmtk::logger().info("mesh is valid");
    m.collapse_qslim(target);
    wmtk::logger().info("collapsed");
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    wmtk::logger().info("runtime in ms{}", duration.count());
}

int main(int argc, char** argv)
{
    CLI::App app{argv[0]};
    std::filesystem::path json_input_file;

    app.add_option(
           "-j",
           json_input_file,
           "JSON input file. See individual components for further instructions.")
        ->required()
        ->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    // read JSON input file
    nlohmann::json json_params;
    try {
        std::ifstream ifs(json_input_file);
        json_params = nlohmann::json::parse(ifs);
    } catch (const std::exception& e) {
        logger().error("Could not load or parse JSON input file");
        logger().error(e.what());
    }

    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, qslim_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, qslim_spec);
    }

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    const std::string input_path = json_params["input"];
    const std::string output = json_params["output"];
    const double env_rel = json_params["eps_rel"];
    const double target_rel = json_params["target_rel"];
    const int num_thread = json_params["num_threads"];
    double target_abs = json_params["target_abs"];

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    const double remove_duplicate_esp = 1e-5;
    std::vector<size_t> modified_nonmanifold_v;
    wmtk::stl_to_manifold_wmtk_input(
        input_path,
        remove_duplicate_esp,
        box_minmax,
        verts,
        tris,
        modified_nonmanifold_v);

    const double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = env_rel * diag;

    igl::Timer timer;
    timer.start();
    QSLIM m(verts, num_thread);
    m.create_mesh(verts.size(), tris, modified_nonmanifold_v, envelope_size);
    assert(m.check_mesh_connectivity_validity());
    wmtk::logger().info("collapsing mesh {}", input_path);
    // int target_verts = verts.size() * target_rel;
    if (target_abs < 0) {
        target_abs = verts.size() * target_rel;
    }

    run_qslim_collapse(input_path, target_abs, output, m);
    timer.stop();
    logger().info("Took {}", timer.getElapsedTimeInSec());
    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    wmtk::logger().info(
        "After_vertices#: {} \n After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());

    const std::string report_file = json_params["report"];
    if (!report_file.empty()) {
        std::ofstream fout(report_file);
        nlohmann::json report;
        report["target_vertices"] = target_abs;
        report["vertices"] = m.vert_capacity();
        report["triangles"] = m.tri_capacity();
        report["time_sec"] = timer.getElapsedTimeInSec();
        fout << std::setw(4) << report;
        fout.close();
    }

    return 0;
}
