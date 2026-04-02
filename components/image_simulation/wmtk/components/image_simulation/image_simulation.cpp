#include "image_simulation.hpp"


#include <memory>
#include <vector>

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include "EmbedSurface.hpp"
#include "ImageSimulationMesh.h"
#include "ImageSimulationMeshTri.hpp"
#include "Parameters.h"
#include "extract_soup.hpp"
#include "read_image_msh.hpp"

#include "image_simulation_spec.hpp"

namespace wmtk::components::image_simulation {


void run_3D(const nlohmann::json& json_params, const InputData& input_data)
{
    using wmtk::utils::resolve_path;
    using Tuple = TetMesh::Tuple;

    const std::filesystem::path root =
        json_params.contains("json_input_file") ? json_params["json_input_file"] : "";

    Parameters params;
    params.output_path = json_params["output"];
    const bool skip_simplify = json_params["skip_simplify"];
    const bool use_sample_envelope = json_params["use_sample_envelope"];
    const int NUM_THREADS = json_params["num_threads"];
    const int max_its = json_params["max_iterations"];

    params.epsr = json_params["eps_rel"];
    params.eps = json_params["eps"];
    params.lr = json_params["length_rel"];
    params.stop_energy = json_params["stop_energy"];
    params.preserve_topology = json_params["preserve_topology"];

    const bool write_vtu = json_params["write_vtu"];

    params.debug_output = json_params["DEBUG_output"];
    params.perform_sanity_checks = json_params["DEBUG_sanity_checks"];

    std::filesystem::path output_filename = params.output_path;

    params.init(input_data.V_input.colwise().minCoeff(), input_data.V_input.colwise().maxCoeff());

    igl::Timer timer;
    timer.start();

    image_simulation::ImageSimulationMesh mesh(params, params.eps, NUM_THREADS);
    // first init envelope
    if (input_data.V_envelope.size() != 0) {
        mesh.init_envelope(input_data.V_envelope, input_data.F_envelope);
    }
    if (input_data.V_input_r.size() == 0) {
        logger().info("Use float input for TetWild");
        mesh.init_from_image(input_data.V_input, input_data.T_input, input_data.T_input_tag);
    } else {
        logger().warn("Use RATIONAL input for TetWild");
        mesh.init_from_image(input_data.V_input_r, input_data.T_input, input_data.T_input_tag);
    }

    auto write_unique_vtu = [&write_vtu, &mesh, &output_filename]() {
        static size_t vtu_counter = 0;
        if (write_vtu) {
            mesh.write_vtu(fmt::format("{}_{}", output_filename.string(), vtu_counter++));
        }
    };

    mesh.consolidate_mesh();
    {
        int num_parts = mesh.flood_fill();
        logger().info("flood fill parts {}", num_parts);
    }

    write_unique_vtu();

    // /////////mesh improvement
    mesh.mesh_improvement(max_its); // <-- tetwild

    mesh.consolidate_mesh();
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);

    {
        int num_parts = mesh.flood_fill();
        logger().info("flood fill parts {}", num_parts);
    }

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
    write_unique_vtu();
    if (write_vtu) {
        mesh.write_surface(output_filename.string() + "_surface.obj");
    }
}

void run_2D(const nlohmann::json& json_params, const InputData& input_data)
{
    using wmtk::utils::resolve_path;
    using Tuple = TetMesh::Tuple;

    const std::filesystem::path root =
        json_params.contains("json_input_file") ? json_params["json_input_file"] : "";

    Parameters params;
    params.output_path = json_params["output"];
    const bool skip_simplify = json_params["skip_simplify"];
    const bool use_sample_envelope = json_params["use_sample_envelope"];
    const int NUM_THREADS = json_params["num_threads"];
    const int max_its = json_params["max_iterations"];

    params.epsr = json_params["eps_rel"];
    params.eps = json_params["eps"];
    params.lr = json_params["length_rel"];
    params.stop_energy = json_params["stop_energy"];
    params.preserve_topology = json_params["preserve_topology"];

    params.smooth_without_envelope = json_params["smooth_without_envelope"];

    params.w_amips = json_params["w_amips"];
    params.w_smooth = json_params["w_smooth"];
    params.w_envelope = json_params["w_envelope"];
    params.w_separate = json_params["w_separate"];

    params.dhat = json_params["dhat"];
    params.dhat_rel = json_params["dhat_rel"];

    const bool write_vtu = json_params["write_vtu"];

    params.debug_output = json_params["DEBUG_output"];
    params.perform_sanity_checks = json_params["DEBUG_sanity_checks"];

    std::filesystem::path output_filename = params.output_path;

    params.init(input_data.V_input.colwise().minCoeff(), input_data.V_input.colwise().maxCoeff());

    igl::Timer timer;
    timer.start();

    image_simulation::tri::ImageSimulationMeshTri mesh(params, params.eps, NUM_THREADS);
    // first init envelope
    if (input_data.V_envelope.size() != 0) {
        mesh.init_envelope(input_data.V_envelope, input_data.F_envelope);
    }
    if (input_data.V_input_r.size() != 0) {
        log_and_throw_error("Input must be float for 2D!");
    }
    mesh.init_from_image(input_data.V_input, input_data.T_input, input_data.T_input_tag);

    auto write_unique_vtu = [&write_vtu, &mesh, &output_filename]() {
        static size_t vtu_counter = 0;
        if (write_vtu) {
            mesh.write_vtu(fmt::format("{}_{}", output_filename.string(), vtu_counter++));
        }
    };

    mesh.consolidate_mesh();

    write_unique_vtu();

    // /////////apply operation
    const std::string operation = json_params["operation"];
    if (operation == "remeshing") {
        mesh.mesh_improvement(max_its); // <-- tetwild
    } else if (operation == "fill_holes_topo") {
        const std::vector<int64_t> fill_holes_tags = json_params["fill_holes_tags"];
        const double raw_threshold = json_params["fill_holes_threshold"];
        const double threshold =
            raw_threshold < 0 ? std::numeric_limits<double>::infinity() : raw_threshold;
        mesh.fill_holes_topo(fill_holes_tags, threshold);
    } else if (operation == "tight_seal_topo") {
        // tight_seal_tag_sets is a list of lists: [[t1,t2],[t3,...]]
        std::vector<std::unordered_set<int64_t>> tag_sets;
        for (const auto& s : json_params["tight_seal_tag_sets"]) {
            std::unordered_set<int64_t> ts;
            for (const auto& v : s) {
                ts.insert(v.get<int64_t>());
            }
            tag_sets.push_back(std::move(ts));
        }
        const double raw_threshold = json_params["tight_seal_threshold"];
        const double threshold =
            raw_threshold < 0 ? std::numeric_limits<double>::infinity() : raw_threshold;
        mesh.tight_seal_topo(tag_sets, threshold);
    } else if (operation == "keep_largest_cc") {
        const std::vector<int64_t> lcc_tags = json_params["keep_largest_cc_tags"];
        mesh.keep_largest_connected_component(lcc_tags);
    } else {
        log_and_throw_error("Unknown image simulation operation");
    }

    mesh.consolidate_mesh();
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);

    /////////output
    auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    std::ofstream fout(output_filename.string() + ".log");
    fout << "#f: " << mesh.get_faces().size() << std::endl;
    fout << "#v: " << mesh.get_vertices().size() << std::endl;
    fout << "max_energy: " << max_energy << std::endl;
    fout << "avg_energy: " << avg_energy << std::endl;
    fout << "eps: " << params.eps << std::endl;
    fout << "threads: " << NUM_THREADS << std::endl;
    fout << "time: " << time << std::endl;
    fout.close();

    wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    mesh.write_msh(output_filename.string() + ".msh");
    write_unique_vtu();
}

void image_simulation(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;
    using Tuple = TetMesh::Tuple;

    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, image_simulation_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, image_simulation_spec);
    }

    const std::filesystem::path root =
        json_params.contains("json_input_file") ? json_params["json_input_file"] : "";

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

    std::filesystem::path output_filename = resolve_path(root, json_params["output"]);

    if (output_filename.has_extension() && output_filename.extension() != ".msh") {
        output_filename.replace_extension(".msh");
        logger().warn(
            "Extension of provided output filename is ignored. Output will be {}",
            output_filename.string());
    }
    output_filename.replace_extension(""); // extension is added back later
    json_params["output"] = output_filename.string(); // propagate resolved path to run_2D/run_3D

    auto get_unique_vtu_name = [&output_filename]() -> std::string {
        static size_t vtu_counter = 0;
        return fmt::format("{}_{}", output_filename.string(), vtu_counter++);
    };

    // read image or .msh
    const InputData input_data =
        (std::filesystem::path(input_paths[0]).extension() == ".msh")
            ? read_image_msh(input_paths[0])
            : read_image(input_paths, output_filename.string(), json_params);

    if (input_data.T_input.cols() == 4) {
        run_3D(json_params, input_data);
    } else {
        run_2D(json_params, input_data);
    }
    wmtk::logger().info("======= finish =========");
}

} // namespace wmtk::components::image_simulation