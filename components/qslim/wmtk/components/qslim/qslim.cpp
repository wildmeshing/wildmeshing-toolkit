#include "qslim.hpp"

#include <igl/Timer.h>
#include <igl/read_triangle_mesh.h>
#include <jse/jse.h>
#include <wmtk/utils/Reader.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include "QSlimMesh.h"
#include "qslim_spec.hpp"

namespace wmtk::components::qslim {

double run_qslim_collapse(std::string input, int target, std::string output, QSlimMesh& m)
{
    wmtk::logger().info("target number of verts: {}", target);
    if (!m.check_mesh_connectivity_validity()) {
        log_and_throw_error("mesh is invalid!");
    }

    igl::Timer timer;
    timer.start();
    m.collapse_qslim(target);
    timer.stop();

    wmtk::logger().info("runtime: {:.4}s", timer.getElapsedTimeInSec());

    return timer.getElapsedTimeInSec();
}

void qslim(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, qslim_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, qslim_spec);
    }

    const std::filesystem::path root =
        json_params.contains("json_input_file") ? json_params["json_input_file"] : "";

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            log_file_name = resolve_path(root, log_file_name).string();
            set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    const std::string input_path = resolve_path(root, json_params["input"]).string();
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
    stl_to_manifold_wmtk_input(
        input_path,
        remove_duplicate_esp,
        box_minmax,
        verts,
        tris,
        modified_nonmanifold_v);

    const double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = env_rel * diag;

    QSlimMesh m(verts, num_thread);
    m.create_mesh(verts.size(), tris, modified_nonmanifold_v, envelope_size);
    assert(m.check_mesh_connectivity_validity());
    logger().info("collapsing mesh {}", input_path);
    if (target_abs < 0) {
        target_abs = verts.size() * target_rel;
    }

    const double runtime = run_qslim_collapse(input_path, target_abs, output, m);
    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    logger().info("After #V = {}, #F = {}", m.vert_capacity(), m.tri_capacity());

    if (json_params["throw_on_fail"]) {
        const size_t nv = m.get_vertices().size();
        if (nv > target_abs) {
            log_and_throw_error("Target not reached: #V = {}, target was {}", target_abs, nv);
        }
    }

    const std::string report_file = json_params["report"];
    if (!report_file.empty()) {
        std::ofstream fout(report_file);
        nlohmann::json report;
        report["target_vertices"] = target_abs;
        report["vertices"] = m.vert_capacity();
        report["triangles"] = m.tri_capacity();
        report["time_sec"] = runtime;
        fout << std::setw(4) << report;
        fout.close();
    }
}

} // namespace wmtk::components::qslim