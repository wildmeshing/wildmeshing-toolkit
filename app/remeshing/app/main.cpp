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
    std::string output,
    UniformRemeshing& m,
    int itrs,
    bool debug_output = false)
{
    auto start = high_resolution_clock::now();
    m.uniform_remeshing(itrs, debug_output);
    // m.consolidate_mesh();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    m.consolidate_mesh();

    std::filesystem::path outp(output);
    std::string base = outp.replace_extension("").string();
    m.write_vtu(output);

    // m.write_triangle_mesh(output);
    // m.write_feature_vertices_obj(output + ".feature_vertices.obj");
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


    std::vector<std::pair<size_t, int>> fixed_vertices;
    std::vector<std::pair<std::array<size_t, 2>, int>> feature_edge_list;

    std::vector<size_t> face_patch_ids;

    try {
        if (json_params.contains("patches")) {
            std::string patches_path = json_params["patches"];
            logger().info("Loading patches info file: {}", patches_path);


            nlohmann::json patches_json;
            std::ifstream cifs(patches_path);
            if (!cifs.is_open()) {
                logger().error("Cannot open patches file {}", patches_path);
            } else {
                cifs >> patches_json;
            }

            const auto& corners = patches_json["corner_vids"];
            fixed_vertices.reserve(corners.size());

            for (const auto& [key, val] : corners.items()) {
                const int corner_ids = std::stoi(key) + 1; // add 1 so 0 means "not frozen"
                const size_t vid = val.get<size_t>();

                fixed_vertices.emplace_back(vid, corner_ids);
            }
            logger().info("Loaded {} fixed (corner) vertices.", fixed_vertices.size());

            const auto& feature_edges = patches_json["edge2seg"];
            feature_edge_list.reserve(feature_edges.size());

            for (const auto& [key, val] : feature_edges.items()) {
                const auto comma_pos = key.find(',');
                if (comma_pos == std::string::npos) {
                    logger().error("Invalid feature edge key (no comma): {}", key);
                    continue;
                }

                const size_t v0 = std::stoul(key.substr(0, comma_pos));
                const size_t v1 = std::stoul(key.substr(comma_pos + 1));

                const int seg_id_plus1 = val.get<int>() + 1; // <-- THIS is the id (+1)

                feature_edge_list.push_back({{{v0, v1}}, seg_id_plus1});
            }

            logger().info("Loaded {} feature edges.", feature_edge_list.size());

            const auto& pids = patches_json["fid2patch"];
            face_patch_ids.resize(pids.size());
            for (const auto& [key, pid] : pids.items()) {
                const size_t fid = std::stoul(key);
                face_patch_ids[fid] = pid;
            }
        }
    } catch (const std::exception& e) {
        log_and_throw_error("Error reading corner vertices: {}", e.what());
    }

    std::vector<size_t> feature_vertices;
    feature_vertices.reserve(feature_edge_list.size() * 2);

    for (const auto& e : feature_edge_list) {
        const auto& ab = e.first;
        feature_vertices.push_back(ab[0]);
        feature_vertices.push_back(ab[1]);
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
    const double length_factor = json_params["length_factor"];
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
    m.set_patch_ids(face_patch_ids);


    if (length_factor < 0) {
        std::vector<double> properties = m.average_len_valen();
        wmtk::logger().info("before remesh properties: {}", properties);
        if (length_abs < 0 && length_rel < 0) {
            logger().info("Use average edge length as target length.");
            length_abs = properties[0];
        } else if (length_abs < 0) {
            length_abs = diag * length_rel;
        }
        logger().info("absolute target length: {}", length_abs);
        m.set_target_edge_length(length_abs);
    } else {
        logger().info("Use per-patch length with factor {}", length_factor);
        m.set_per_patch_target_edge_length(length_factor);
    }

    timer.start();
    run_remeshing(input_path, output, m, itrs, debug_output);
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