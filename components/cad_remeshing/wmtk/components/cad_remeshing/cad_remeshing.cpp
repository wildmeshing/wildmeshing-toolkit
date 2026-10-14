#include "cad_remeshing.hpp"

#include <wmtk/utils/io.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <igl/Timer.h>
#include <igl/read_triangle_mesh.h>
#include <jse/jse.h>

#include <cad_remeshing_spec.hpp>
#include "CadRemeshing.h"

namespace wmtk::components::cad_remeshing {

void run_remeshing(
    const std::string& output,
    const std::string& report_file,
    CadRemeshing& m,
    int itrs,
    bool debug_output = false)
{
    m.uniform_remeshing(itrs, debug_output);
    m.consolidate_mesh();
    m.write_vtu(output, report_file);
}

void cad_remeshing(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        const auto& spec = jse::embed::wmtk_cad_remeshing_spec::cad_remeshing_spec::spec();
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
    }

    const std::filesystem::path root = json_params["input_dir"];

    std::vector<std::pair<size_t, int>> fixed_vertices;
    std::vector<std::pair<std::array<size_t, 2>, int>> feature_edge_list;

    std::vector<size_t> face_patch_ids;

    try {
        const std::string patches_path = resolve_path(root, json_params["patches"]).string();
        logger().info("Loading patches info file: {}", patches_path);

        nlohmann::json patches_json;
        std::ifstream cifs(patches_path);
        if (!cifs.is_open()) {
            log_and_throw_error("Cannot open patches file {}", patches_path);
        }
        cifs >> patches_json;

        // A missing key would read as null and iterate as empty -- a patches file without
        // feature edges would silently remesh without features.
        for (const char* key : {"fid2patch", "edge2seg", "corner_vids"}) {
            if (!patches_json.contains(key) || !patches_json[key].is_object()) {
                log_and_throw_error("Patches file {} has no object \"{}\"", patches_path, key);
            }
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

        // fid2patch must map every face 0..n-1 exactly once; set_patch_ids checks the count
        // against the mesh, the range and duplicates are checked here.
        const auto& pids = patches_json["fid2patch"];
        constexpr size_t unset = std::numeric_limits<size_t>::max();
        face_patch_ids.assign(pids.size(), unset);
        for (const auto& [key, pid] : pids.items()) {
            const size_t fid = std::stoul(key);
            if (fid >= face_patch_ids.size()) {
                log_and_throw_error(
                    "fid2patch: face id {} is out of range for {} entries; the map must contain "
                    "every face 0..{} exactly once",
                    fid,
                    pids.size(),
                    pids.size() - 1);
            }
            if (face_patch_ids[fid] != unset) {
                log_and_throw_error("fid2patch: face id {} appears more than once", fid);
            }
            face_patch_ids[fid] = pid.get<size_t>();
        }
    } catch (const std::exception& e) {
        log_and_throw_error("Error reading the patches file: {}", e.what());
    }

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            log_file_name = resolve_path(root, log_file_name).string();
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    const std::string input_path = resolve_path(root, json_params["input"]).string();
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
    const std::string report_file = json_params["report"];

    // Avoid confusion: if the report already exists, delete it and write a fresh one.
    if (!report_file.empty() && std::filesystem::exists(report_file)) {
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
    VF_to_vectors(inV, inF, verts, tris);

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
    // Envelope thickness: eps if set, else eps_rel relative to the bounding box diagonal if set,
    // else (both negative, the default) half the average input edge length -- decided below,
    // once the mesh exists.
    if (eps < 0 && eps_rel > 0) {
        eps = eps_rel * diag;
    }
    wmtk::logger().info("Mesh diag: {}", diag);
    igl::Timer timer;

    wmtk::logger().info("Total frozen vertices: {}", fixed_vertices.size());

    CadRemeshing m(verts, num_threads, !sample_envelope);
    m.set_feature_edges(feature_edge_list);
    m.create_mesh(verts.size(), tris, fixed_vertices, freeze_boundary);
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
    auto properties = m.average_len_valen();
    report["before"]["avg_length"] = properties[0];
    report["before"]["max_length"] = properties[1];
    report["before"]["min_length"] = properties[2];
    report["before"]["avg_valence"] = properties[3];
    report["before"]["max_valence"] = properties[4];
    report["before"]["min_valence"] = properties[5];
    // init envelopes
    {
        if (eps < 0) {
            eps = properties[0] * 0.5;
            logger().info("Envelope thickness eps: {} (half the average input edge length)", eps);
        } else {
            logger().info("Envelope thickness eps: {}", eps);
        }
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
    if (!report_file.empty()) {
        std::ofstream fout(report_file);
        fout << std::setw(4) << report;
    }

    timer.start();
    run_remeshing(output, report_file, m, itrs, debug_output);
    timer.stop();

    if (report_file.empty()) {
        return;
    }

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
}

} // namespace wmtk::components::cad_remeshing
