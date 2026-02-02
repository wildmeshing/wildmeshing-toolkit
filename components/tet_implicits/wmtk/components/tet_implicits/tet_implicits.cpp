#include "tet_implicits.hpp"

#include <vector>

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include "Parameters.h"
#include "TetImplicitsMesh.h"
#include "read_image_msh.hpp"

#include "tet_implicits_spec.hpp"

// Enables passing Eigen matrices to fmt/spdlog.
template <typename T>
struct fmt::formatter<T, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>, char>>
    : ostream_formatter
{
};


namespace wmtk::components::tet_implicits {

void tet_implicits(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;
    using Tuple = TetMesh::Tuple;

    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, tet_implicits_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, tet_implicits_spec);
    }

    const std::filesystem::path root = json_params["json_input_file"];

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            log_file_name = resolve_path(root, log_file_name).string();
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    std::vector<std::string> input_paths = json_params["input"];
    for (std::string& p : input_paths) {
        p = resolve_path(root, p).string();
    }

    if (std::filesystem::path(input_paths[0]).extension() != ".msh") {
        log_and_throw_error("Input must be a .msh file.");
    }

    Parameters params;
    params.output_path = resolve_path(root, json_params["output"]).string();
    int NUM_THREADS = json_params["num_threads"];
    const std::string op = json_params["operation"];

    params.input_tags = json_params["input_tags"];
    params.output_tags = json_params["output_tags"];
    params.epsr = json_params["eps_rel"];
    params.eps = json_params["eps"];
    params.dr = json_params["d_rel"];
    params.d = json_params["d"];
    params.preserve_topology = json_params["preserve_topology"];

    const bool write_vtu = json_params["write_vtu"];

    params.debug_output = json_params["DEBUG_output"];
    params.perform_sanity_checks = json_params["DEBUG_sanity_checks"];

    std::filesystem::path output_filename = params.output_path;

    if (output_filename.has_extension() && output_filename.extension() != ".msh") {
        output_filename.replace_extension(".msh");
        logger().warn(
            "Extension of provided output filename is ignored. Output will be {}",
            output_filename.string());
    }
    output_filename.replace_extension(""); // extension is added back later

    auto get_unique_vtu_name = [&output_filename]() -> std::string {
        static size_t vtu_counter = 0;
        return fmt::format("{}_{}", output_filename.string(), vtu_counter++);
    };

    // read image / MSH
    MatrixXd V_input;
    MatrixXi T_input;
    MatrixXi T_input_tag;

    MatrixXd V_envelope;
    MatrixXi F_envelope;

    // input is a tet mesh
    read_image_msh(input_paths[0], V_input, T_input, T_input_tag, V_envelope, F_envelope);

    params.init(V_input.colwise().minCoeff(), V_input.colwise().maxCoeff());
    logger().info(
        "===== Params =====\n  operation = {}\n  input_tags = {}\n  output_tags = {}\n  d = {}\n",
        op,
        params.input_tags,
        params.output_tags,
        params.d);

    igl::Timer timer;
    timer.start();

    tet_implicits::TetImplicitsMesh mesh(params, NUM_THREADS);

    mesh.init_from_image(V_input, T_input, T_input_tag);

    mesh.consolidate_mesh();

    if (write_vtu) mesh.write_vtu(get_unique_vtu_name());

    // implicits
    if (op == "separate") {
        logger().info("Separate tags {} by distance {}", params.input_tags, params.d);
        mesh.op_separate();
    } else if (op == "tight_seal") {
        logger().info("Tight seal tags {} within distance {}", params.input_tags, params.d);
        mesh.op_tight_seal();
    }

    mesh.consolidate_mesh();
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);


    /////////output
    auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    std::ofstream fout(output_filename.string() + ".log");
    fout << "#t: " << mesh.tet_size() << std::endl;
    fout << "#v: " << mesh.vertex_size() << std::endl;
    fout << "max_energy: " << max_energy << std::endl;
    fout << "avg_energy: " << avg_energy << std::endl;
    fout << "eps: " << params.eps << std::endl;
    fout << "threads: " << NUM_THREADS << std::endl;
    fout << "time: " << time << std::endl;
    fout.close();

    wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    mesh.write_msh(output_filename.string() + ".msh");
    if (write_vtu) mesh.write_vtu(get_unique_vtu_name());
    // if (write_vtu) mesh.write_surface(output_filename.string() + "_surface.obj");

    wmtk::logger().info("======= finish =========");
}

} // namespace wmtk::components::tet_implicits