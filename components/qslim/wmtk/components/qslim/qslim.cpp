#include "qslim.hpp"
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/utils/Preallocation.hpp>
#include "qslim.hpp"

#include <igl/Timer.h>
#include <jse/jse.h>
#include <wmtk/Types.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <qslim_spec.hpp>
#include "QSlimMesh.h"

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

static void run(nlohmann::json json_params, io::ComponentIO& component_io)
{
    component_io.prepare(json_params, "qslim");
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        const auto spec = jse::embed::wmtk_qslim_spec::qslim_spec::spec();
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
            set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    const std::string input_path = component_io.path(root, json_params["input"]);
    const std::string output = json_params["output"];
    const double env_rel = json_params["eps_rel"];
    const double boundary_env_rel = json_params["boundary_eps_rel"];
    const double target_rel = json_params["target_rel"];
    const int num_thread = json_params["num_threads"];
    double target_abs = json_params["target_abs"];

    const double remove_duplicate_eps = 1e-5;
    MatrixXd V;
    MatrixXi F;
    component_io.triangles(input_path, V, F, remove_duplicate_eps);

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    std::vector<size_t> modified_nonmanifold_v;

    box_minmax.first = V.colwise().minCoeff();
    box_minmax.second = V.colwise().maxCoeff();
    VF_to_vectors(V, F, verts, tris);


    const double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = env_rel * diag;
    const double boundary_envelope_size = boundary_env_rel > 0 ? boundary_env_rel * diag : 0;
    if (boundary_envelope_size > 0) {
        logger().info("boundary envelope eps {:.6}", boundary_envelope_size);
    }

    QSlimMesh m(verts, num_thread);
    wmtk::set_preallocation_factor_from_json(m, json_params);
    m.create_mesh(
        verts.size(),
        tris,
        modified_nonmanifold_v,
        envelope_size,
        boundary_envelope_size);
    assert(m.check_mesh_connectivity_validity());
    logger().info("collapsing mesh {}", input_path);
    if (target_abs < 0) {
        target_abs = verts.size() * target_rel;
    }

    const double runtime = run_qslim_collapse(input_path, target_abs, output, m);
    m.consolidate_mesh();
    if (!output.empty()) m.write_triangle_mesh(output);
    logger().info("After #V = {}, #F = {}", m.vert_capacity(), m.tri_capacity());

    if (json_params["throw_on_fail"]) {
        const size_t nv = m.get_vertices().size();
        if (nv > target_abs) {
            log_and_throw_error("Target not reached: #V = {}, target was {}", target_abs, nv);
        }
    }

    const std::string report_file = json_params["report"];
    if (component_io.memory() || !report_file.empty()) {
        nlohmann::json report;
        report["target_vertices"] = target_abs;
        report["vertices"] = m.vert_capacity();
        report["triangles"] = m.tri_capacity();
        report["time_sec"] = runtime;
        component_io.result.second = report;
        if (!component_io.memory()) {
            std::ofstream fout(report_file);
            fout << std::setw(4) << report;
        }
    }
    if (component_io.memory())
        component_io.result.first["mesh"] =
            io::triangle_data(m, [&](size_t v) { return m.vertex_attrs[v].pos; });
}

void qslim(nlohmann::json json_params)
{
    io::ComponentIO context;
    run(std::move(json_params), context);
}
ComponentResult qslim(nlohmann::json json_params, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(json_params), context);
    return std::move(context.result);
}

} // namespace wmtk::components::qslim