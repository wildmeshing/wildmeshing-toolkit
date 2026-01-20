#include <remeshing/UniformRemeshing.h>

#include <CLI/CLI.hpp>

#include <wmtk/utils/Reader.hpp>
#include <wmtk/utils/io.hpp>

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
    // wmtk::logger().info(
    //     "Before_vertices#: {} \n Before_tris#: {}",
    //     m.vert_capacity(),
    //     m.tri_capacity());

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
    // wmtk::logger().info("runtime in ms {}", duration.count());
    // wmtk::logger().info("current_memory {}", getCurrentRSS() / (1024. * 1024));
    // wmtk::logger().info("peak_memory {}", getPeakRSS() / (1024. * 1024));
    // wmtk::logger().info("after remesh properties: {}", properties);
    // wmtk::logger().info(
    //     "After_vertices#: {} \n\t After_tris#: {}",
    //     m.vert_capacity(),
    //     m.tri_capacity());
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
    double eps = json_params["eps"];
    const double eps_rel = json_params["eps_rel"];
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
    // create report file
    auto report_file = output + "_report.json";

    // Avoid confusion: if the report already exists, delete it and write a fresh one.
    if (std::filesystem::exists(report_file)) {
        std::filesystem::remove(report_file);
    }

    nlohmann::json report = nlohmann::json::object();
    report["before"] = nlohmann::json::object();
    report["after"] = nlohmann::json::object();
    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;

    Eigen::MatrixXd inV;
    Eigen::MatrixXi inF;
    if (std::filesystem::path(input_path).extension() == ".msh") {
        MshData msh;
        msh.load(input_path);
        inV.resize(msh.get_num_face_vertices(), 3);
        inF.resize(msh.get_num_faces(), 3);
        msh.extract_face_vertices(
            [&inV](size_t i, double x, double y, double z) { inV.row(i) << x, y, z; });
        msh.extract_faces(
            [&inF](size_t i, size_t v0, size_t v1, size_t v2) { inF.row(i) << v0, v1, v2; });
    } else {
        igl::read_triangle_mesh(input_path, inV, inF);
    }
    verts.resize(inV.rows());
    tris.resize(inF.rows());
    wmtk::eigen_to_wmtk_input(verts, tris, inV, inF);

    {
        // basic mesh size stats
        report["before"]["nV"] = static_cast<int64_t>(inV.rows());
        report["before"]["nF"] = static_cast<int64_t>(inF.rows());

        // min/max internal angle
        {
            Eigen::MatrixXd angles;
            igl::internal_angles(inV, inF, angles);
            auto min_angle = angles.minCoeff();
            auto max_angle = angles.maxCoeff();
            logger().info("Before Min angle: {}, Max angle: {}", min_angle, max_angle);
            report["before"]["min_angle"] = min_angle;
            report["before"]["max_angle"] = max_angle;
            report["before"]["avg_angle"] = angles.mean();
        }
        // min/max doublearea
        {
            Eigen::VectorXd double_areas;
            igl::doublearea(inV, inF, double_areas);
            auto min_da = double_areas.minCoeff();
            auto max_da = double_areas.maxCoeff();
            logger().info("Before Min double area: {}, Max double area: {}", min_da, max_da);
            report["before"]["min_da"] = min_da;
            report["before"]["max_da"] = max_da;
            report["before"]["avg_da"] = double_areas.mean();
        }
    }

    box_minmax = std::pair(inV.colwise().minCoeff(), inV.colwise().maxCoeff());
    double diag = (box_minmax.first - box_minmax.second).norm();
    if (eps < 0) {
        eps = eps_rel * diag;
    }
    wmtk::logger().info("Mesh diag: {}", diag);
    wmtk::logger().info("Relative eps: {}", eps_rel);
    wmtk::logger().info("Envelope thickness eps: {}", eps);
    igl::Timer timer;

    wmtk::logger().info("Total frozen vertices: {}", fixed_vertices.size());

    UniformRemeshing m(verts, num_threads, !sample_envelope);
    m.set_feature_edges(feature_edge_list);
    // m.set_feature_vertices(feature_vertices);
    m.create_mesh(verts.size(), tris, fixed_vertices, freeze_boundary, eps);
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

        // wmtk::logger().info("Mesh diag: {}", diag);
        // wmtk::logger().info("Relative eps: {}", eps_rel);
        // wmtk::logger().info("Envelope thickness eps: {}", eps);
    } else {
        logger().info("Use per-patch length with factor {}", length_factor);
        m.set_per_patch_target_edge_length(length_factor);
    }
    auto properties = m.average_len_valen();
    report["before"]["avg_length"] = properties[0];
    report["before"]["max_length"] = properties[1];
    report["before"]["min_length"] = properties[2];
    report["before"]["avg_valence"] = properties[3];
    report["before"]["max_valence"] = properties[4];
    report["before"]["min_valence"] = properties[5];
    // init envelopes
    {
        eps = properties[0] * 0.5; // reset eps to half of avg edge length
        const auto feature_edges =
            m.get_edges_by_condition([](const EdgeAttributes& e) { return e.is_feature; });

        // Convert inV from Eigen::MatrixXd to std::vector<Eigen::Vector3d>
        std::vector<Eigen::Vector3d> V_vec(inV.rows());
        for (int i = 0; i < inV.rows(); ++i) {
            V_vec[i] = inV.row(i);
        }

        // Convert inF from Eigen::MatrixXi to std::vector<Eigen::Vector3i>
        std::vector<Eigen::Vector3i> F_vec(inF.rows());
        for (int i = 0; i < inF.rows(); ++i) {
            F_vec[i] = inF.row(i);
        }

        // Convert feature edges to std::vector<Eigen::Vector2i>
        std::vector<Eigen::Vector2i> E_vec(feature_edges.size());
        for (int i = 0; i < feature_edges.size(); ++i) {
            E_vec[i] = Eigen::Vector2i(feature_edges[i][0], feature_edges[i][1]);
        }

        if (eps > 0) {
            m.m_envelope.init(V_vec, F_vec, eps);
            if (!feature_edges.empty()) {
                m.m_feature_envelope.init(V_vec, E_vec, eps);
            }
            m.m_has_envelope = true;
        } else {
            m.m_envelope.init(V_vec, F_vec, 0.0);
            if (!feature_edges.empty()) {
                m.m_feature_envelope.init(V_vec, E_vec, 0.0);
            }
        }
    }
    // Write an initial report so downstream code (e.g. write_vtu inside run_remeshing)
    // can update/append fields like after min/max angle and double-area.
    {
        std::ofstream fout(report_file);
        fout << std::setw(4) << report;
    }

    timer.start();
    run_remeshing(input_path, output, m, itrs, debug_output);
    timer.stop();

    // Reload report in case downstream code (e.g. write_vtu) has updated it.
    try {
        std::ifstream fin(report_file);
        if (fin) {
            fin >> report;
        }
    } catch (const std::exception& e) {
        logger().warn("Failed to reload report file {}: {}", report_file, e.what());
    }

    if (!report.is_object()) {
        report = nlohmann::json::object();
    }
    if (!report.contains("before") || !report["before"].is_object()) {
        report["before"] = nlohmann::json::object();
    }
    if (!report.contains("after") || !report["after"].is_object()) {
        report["after"] = nlohmann::json::object();
    }

    properties = m.average_len_valen();
    report["after"]["nV"] = static_cast<int64_t>(m.vert_capacity());
    report["after"]["nF"] = static_cast<int64_t>(m.tri_capacity());
    report["after"]["avg_length"] = properties[0];
    report["after"]["max_length"] = properties[1];
    report["after"]["min_length"] = properties[2];
    report["after"]["avg_valence"] = properties[3];
    report["after"]["max_valence"] = properties[4];
    report["after"]["min_valence"] = properties[5];

    report["time_sec"] = timer.getElapsedTimeInSec();

    // Persist final merged report.
    {
        std::ofstream fout(report_file);
        fout << std::setw(4) << report;
    }

    return 0;
}