#include <remeshing/UniformRemeshing.h>

#include <CLI/CLI.hpp>

#include <wmtk/utils/Reader.hpp>

#include <igl/Timer.h>
#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <jse/jse.h>


#include <stdlib.h>
#include <chrono>
#include <cstdlib>
#include <iostream>

using namespace wmtk;
using namespace app::remeshing;
using namespace std::chrono;

#include <wmtk/utils/getRSS.h>

#include "remeshing_spec.hpp"

void run_remeshing(std::string input, double len, std::string output, UniformRemeshing& m, int itrs)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target len: {}", len);
    m.uniform_remeshing(len, itrs);
    // m.consolidate_mesh();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    auto properties = m.average_len_valen();
    wmtk::logger().info("runtime in ms {}", duration.count());
    wmtk::logger().info("current_memory {}", getCurrentRSS() / (1024. * 1024));
    wmtk::logger().info("peak_memory {}", getPeakRSS() / (1024. * 1024));
    wmtk::logger().info("after remesh properties: {}", properties);
    wmtk::logger().info(
        "After_vertices#: {} \n\t After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
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
        bool r = spec_engine.verify_json(json_params, remeshing_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, remeshing_spec);
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
    const double length_rel = json_params["length_rel"];
    const int num_threads = json_params["num_threads"];
    const int itrs = json_params["max_iterations"];
    const bool sample_envelope = json_params["use_sample_envelope"];
    const bool freeze_boundary = json_params["freeze_boundary"];
    double length_abs = json_params["length_abs"];

    wmtk::logger().info("remeshing on {}", input_path);
    wmtk::logger().info("freeze bnd {}", freeze_boundary);
    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    double remove_duplicate_eps = 1e-5;
    std::vector<size_t> modified_nonmanifold_v;
    wmtk::stl_to_manifold_wmtk_input(
        input_path,
        remove_duplicate_eps,
        box_minmax,
        verts,
        tris,
        modified_nonmanifold_v);

    double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = env_rel * diag;
    igl::Timer timer;

    UniformRemeshing m(verts, num_threads, !sample_envelope);
    m.create_mesh(verts.size(), tris, modified_nonmanifold_v, freeze_boundary, envelope_size);

    {
        std::vector<double> properties = m.average_len_valen();
        wmtk::logger().info("before remesh properties: {}", properties);
        if (length_abs < 0 && length_rel < 0) {
            logger().info("Use average edge length as target length.");
            length_abs = properties[0];
        } else if (length_abs < 0) {
            length_abs = diag * length_rel;
        }
    }
    logger().info("absolute target length: {}", length_abs);

    timer.start();
    run_remeshing(input_path, length_abs, output, m, itrs);
    timer.stop();

    const std::string report_file = json_params["report"];
    if (!report_file.empty()) {
        std::ofstream fout(report_file);
        nlohmann::json report;
        std::vector<double> properties = m.average_len_valen();
        report["avg_length"] = properties[0];
        report["max_length"] = properties[1];
        report["min_length"] = properties[2];
        report["avg_valence"] = properties[3];
        report["max_valence"] = properties[4];
        report["min_valence"] = properties[5];
        report["target_length"] = length_abs;
        report["time_sec"] = timer.getElapsedTimeInSec();
        fout << std::setw(4) << report;
        fout.close();
    }

    return 0;
}
