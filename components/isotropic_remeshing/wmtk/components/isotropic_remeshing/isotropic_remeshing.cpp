#include "isotropic_remeshing.hpp"
#include <wmtk/utils/Preallocation.hpp>

#include <igl/Timer.h>
#include <jse/jse.h>

#include <wmtk/utils/getRSS.h>
#include <wmtk/Types.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <isotropic_remeshing_spec.hpp>
#include "IsotropicRemeshing.h"

namespace wmtk::components::isotropic_remeshing {

double
run_remeshing(std::string input, double len, std::string output, IsotropicRemeshing& m, int itrs)
{
    igl::Timer timer;
    wmtk::logger().info("target len: {}", len);
    timer.start();
    m.uniform_remeshing(len, itrs);
    timer.stop();

    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    auto properties = m.average_len_valen();
    wmtk::logger().info("runtime {:.4}s", timer.getElapsedTimeInSec());
    wmtk::logger().info("current_memory {}", getCurrentRSS() / (1024. * 1024));
    wmtk::logger().info("peak_memory {}", getPeakRSS() / (1024. * 1024));
    wmtk::logger().info("after remesh properties: {}", properties);
    wmtk::logger().info(
        "After_vertices#: {} \n\t After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());

    return timer.getElapsedTimeInSec();
}

void isotropic_remeshing(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        const auto spec =
            jse::embed::wmtk_isotropic_remeshing_spec::isotropic_remeshing_spec::spec();
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
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    const std::string input_path = resolve_path(root, json_params["input"]).string();
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
    std::vector<size_t> modified_nonmanifold_v;
    {
        double remove_duplicate_eps = 1e-5;
        MatrixXd V;
        MatrixXi F;
        wmtk::io::read_triangle_mesh(input_path, V, F, remove_duplicate_eps);
        box_minmax.first = V.colwise().minCoeff();
        box_minmax.second = V.colwise().maxCoeff();
        VF_to_vectors(V, F, verts, tris);
    }

    double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = env_rel * diag;
    igl::Timer timer;

    IsotropicRemeshing m(verts, num_threads, !sample_envelope);
    wmtk::set_preallocation_factor_from_json(m, json_params);
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


    const double runtime = run_remeshing(input_path, length_abs, output, m, itrs);

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
        report["time_sec"] = runtime;
        fout << std::setw(4) << report;
        fout.close();
    }
}

} // namespace wmtk::components::isotropic_remeshing