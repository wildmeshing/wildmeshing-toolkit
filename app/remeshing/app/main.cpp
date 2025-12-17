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

void run_remeshing(
    std::string input,
    double len,
    std::string output,
    UniformRemeshing& m,
    int itrs,
    bool debug_output = false)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target len: {}", len);
    m.uniform_remeshing(len, itrs, debug_output);
    // m.consolidate_mesh();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    m.write_feature_vertices_obj(output + ".feature_vertices.obj");
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
        log_and_throw_error("Could not load or parse JSON input file: {}", e.what());
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


    std::vector<size_t> fixed_vertices;

    try {
        if (json_params.contains("corner")) {
            std::string corner_path = json_params["corner"].get<std::string>();
            logger().info("Loading corner file: {}", corner_path);


            nlohmann::json corner_json;
            std::ifstream cifs(corner_path);
            if (!cifs.is_open()) {
                logger().error("Cannot open corner file {}", corner_path);
            } else {
                cifs >> corner_json;
            }

            for (const auto& val : corner_json) {
                size_t vid;

                if (val.is_number_integer() || val.is_number_unsigned()) {
                    vid = val.get<size_t>();
                } else if (val.is_string()) {
                    vid = std::stoull(val.get<std::string>());
                } else {
                    logger().error("Corner entry has invalid type: {}", val.dump());
                    continue;
                }

                fixed_vertices.push_back(vid);
            }

            logger().info("Loaded fixed vertices: {}", fixed_vertices);
        }
    } catch (const std::exception& e) {
        log_and_throw_error("Error reading corner vertices: {}", e.what());
    }

    nlohmann::json feature_edges;
    std::vector<std::array<size_t, 2>> feature_edge_list;

    try {
        const std::string feature_edges_path = json_params["feature_edges"];
        std::ifstream feifs(feature_edges_path);
        if (!feifs.is_open()) {
            logger().error("Could not open feature_edges file: {}", feature_edges_path);
        } else {
            feifs >> feature_edges;
            logger().info("Loaded feature_edges data from {}", feature_edges_path);

            feature_edge_list.reserve(feature_edges.size());

            for (const auto& [key, val] : feature_edges.items()) {
                const auto comma_pos = key.find(',');
                if (comma_pos == std::string::npos) {
                    logger().error("Invalid feature edge key (no comma): {}", key);
                    continue;
                }

                const std::string_view s0(key.data(), comma_pos);
                const std::string_view s1(key.data() + comma_pos + 1, key.size() - comma_pos - 1);

                const size_t v0 = static_cast<size_t>(std::stoul(std::string(s0)));
                const size_t v1 = static_cast<size_t>(std::stoul(std::string(s1)));

                feature_edge_list.push_back({{v0, v1}});
            }
        }
    } catch (const std::exception& e) {
        log_and_throw_error("Could not load or parse feature_edges JSON file: {}", e.what());
    }

    std::vector<size_t> feature_vertices;
    feature_vertices.reserve(feature_edge_list.size() * 2);
    for (const auto& e : feature_edge_list) {
        feature_vertices.push_back(e[0]);
        feature_vertices.push_back(e[1]);
    }
    wmtk::vector_unique(feature_vertices);

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
    const bool debug_output = json_params["DEBUG_output"];

    wmtk::logger().info("remeshing on {}", input_path);
    wmtk::logger().info("freeze bnd {}", freeze_boundary);
    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;

    Eigen::MatrixXd inV;
    Eigen::MatrixXi inF;
    igl::read_triangle_mesh(input_path, inV, inF);
    verts.resize(inV.rows());
    tris.resize(inF.rows());
    wmtk::eigen_to_wmtk_input(verts, tris, inV, inF);

    box_minmax = std::pair(inV.colwise().minCoeff(), inV.colwise().maxCoeff());
    double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = env_rel * diag;
    igl::Timer timer;

    wmtk::logger().info("Total frozen vertices: {}", fixed_vertices.size());

    UniformRemeshing m(verts, num_threads, !sample_envelope);
    m.set_feature_edges(feature_edge_list);
    // m.set_feature_vertices(feature_vertices);
    m.create_mesh(verts.size(), tris, fixed_vertices, freeze_boundary, envelope_size);

    for (size_t v : fixed_vertices) {
        const auto& attr = m.vertex_attrs[v];
        if (!m.vertex_attrs[v].is_freeze) {
            log_and_throw_error("vertex {} is not frozen but is in fixed_vertices", v);
        }
    }

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

    // to check if fixed vertices are really fixed
    std::unordered_map<size_t, Eigen::Vector3d> original_pos;
    for (size_t v : fixed_vertices) {
        original_pos[v] = verts[v];
    }


    timer.start();
    run_remeshing(input_path, length_abs, output, m, itrs, debug_output);
    timer.stop();

    // to check the position change of fixed vertices
    double max_movement = 0.0;
    for (size_t v : fixed_vertices) {
        const auto& attr = m.vertex_attrs[v];
        Eigen::Vector3d after = attr.pos;
        Eigen::Vector3d before = original_pos[v];
        double dist = (after - before).norm();
        max_movement = std::max(max_movement, dist);
        logger().info("fixed vertex {} moved by {}", v, dist);
    }

    logger().info("Max movement among fixed vertices = {}", max_movement);

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
