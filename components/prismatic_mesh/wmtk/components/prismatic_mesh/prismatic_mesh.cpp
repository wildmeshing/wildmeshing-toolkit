#include "prismatic_mesh.hpp"
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/DataWriter.hpp>

#include <jse/jse.h>
#include <algorithm>
#include <prismatic_mesh_spec.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

namespace wmtk::components::prismatic_mesh {

static void run(nlohmann::json json_params, io::ComponentIO& component_io)
{
    component_io.prepare(json_params, "prismatic_mesh");
    if (component_io.memory()) json_params["output"] = "mesh.vtu";
    const auto spec = jse::embed::wmtk_prismatic_mesh_spec::prismatic_mesh_spec::spec();
    jse::JSE spec_engine;
    if (!spec_engine.verify_json(json_params, spec)) {
        log_and_throw_error(spec_engine.log2str());
    }

    json_params = spec_engine.inject_defaults(json_params, spec);
    const auto path = std::filesystem::path(component_io.path(
        json_params["input_dir"].get<std::string>(),
        json_params["input"].get<std::string>()));
    const auto output_name = json_params["output"].get<std::string>();
    if (output_name.empty()) log_and_throw_error("Output filename must not be empty.");
    auto output_path = std::filesystem::path(
        component_io.path(json_params["input_dir"].get<std::string>(), output_name));
    if (!output_path.has_extension()) output_path += ".vtu";
    if (output_path.extension() != ".vtu" && output_path.extension() != ".msh") {
        log_and_throw_error("Prismatic mesh output must be a .vtu or .msh file.");
    }
    std::vector<std::filesystem::path> extra_outputs;
    if (json_params["export_input_and_shell"].get<bool>())
        for (const auto* extension : {".msh", ".vtu"})
            extra_outputs.push_back(
                output_path.parent_path() /
                (output_path.stem().string() + "_input_shell" + extension));
    auto output_paths = extra_outputs;
    output_paths.push_back(output_path);
    if (!component_io.memory())
        for (const auto& out : output_paths) {
            if (std::filesystem::weakly_canonical(path) == std::filesystem::weakly_canonical(out))
                log_and_throw_error("Input and output must be different files.");
        }
    auto input = component_io.memory() ? load_prismatic_data(component_io.mesh(path.string()))
                                       : load_prismatic_mesh(path);
    logger().info("Loaded prismatic mesh: {}", path.string());
    logger().info(
        "{} vertices, {} tetrahedra; {} input-labeled vertices in full mesh (including volume "
        "interior), {} offset vertices with correspondence",
        input.vertices.rows(),
        input.tetrahedra.rows(),
        input.input_vertices.size(),
        input.offset_vertices.size());
    logger().info(
        "Input cells: {}; offset band cells: {}",
        std::count(input.input_cells.begin(), input.input_cells.end(), 1),
        std::count(input.offset_tet_tags.begin(), input.offset_tet_tags.end(), 1));
    OptimizationOptions optimization;
    optimization.iterations = json_params["iterations"].get<int>();
    optimization.keep_background_mesh = json_params["keep_background_mesh"].get<bool>();
    optimization.min_tet_volume = json_params["min_tet_volume"].get<double>();
    optimization.smoothing_max_backtracks = json_params["smoothing_max_backtracks"].get<int>();
    auto& jacobian = optimization.jacobian_smoothing;
    jacobian.iterations = json_params["jacobian_smoothing_iterations"].get<int>();
    jacobian.target = json_params["jacobian_target"].get<double>();
    jacobian.position_weight = json_params["jacobian_position_weight"].get<double>();
    jacobian.max_step_ratio = json_params["jacobian_max_step_ratio"].get<double>();
    jacobian.max_displacement_ratio = json_params["jacobian_max_displacement_ratio"].get<double>();
    auto& background = optimization.background_remeshing;
    background.enabled = json_params["background_remeshing"].get<bool>();
    background.passes = json_params["background_remeshing_passes"].get<int>();
    background.quality_threshold = json_params["background_quality_threshold"].get<double>();
    background.max_operations = json_params["background_remeshing_max_operations"].get<int>();
    background.max_attempts = json_params["background_remeshing_max_attempts"].get<int>();
    prism_main(input, json_params["thicknessratio"].get<double>(), optimization);
    std::optional<PrismDominantMesh> hybrid;
    if (json_params["prism_dominant"].get<bool>())
        hybrid = build_prism_dominant_mesh(input, optimization.min_tet_volume);
    if (component_io.memory()) {
        MeshMap raw;
        nlohmann::json hybrid_report;
        write_prismatic_mesh(
            input,
            "mesh.vtu",
            hybrid ? &*hybrid : nullptr,
            true,
            &raw,
            &hybrid_report);
        component_io.result.first = io::named_outputs(std::move(raw));
        component_io.result.second = {
            {"jacobian_smoothing", input.jacobian_smoothing_report},
            {"background_remeshing", input.background_remeshing_report}};
        if (hybrid) component_io.result.second["hybrid"] = std::move(hybrid_report);
    } else
        write_prismatic_mesh(input, output_path, hybrid ? &*hybrid : nullptr);
    logger().info("Wrote result mesh: {}", output_path.string());
    for (const auto& out : extra_outputs) {
        if (component_io.memory()) {
            // One result for the .msh and .vtu pair: the mesh, its offset faces and its
            // export report, as the files hold them.
            if (!component_io.result.first.count("input_and_shell")) {
                MeshMap raw;
                nlohmann::json hybrid_report;
                write_prismatic_mesh(
                    input,
                    "mesh.vtu",
                    hybrid ? &*hybrid : nullptr,
                    false,
                    &raw,
                    &hybrid_report);
                auto keys = io::default_output_keys();
                keys["mesh.vtu"] = "input_and_shell";
                keys["mesh_offset_faces.vtu"] = "input_and_shell_offset";
                component_io.result.first.merge(io::named_outputs(std::move(raw), keys));
                if (hybrid)
                    component_io.result.second["input_and_shell_hybrid"] = std::move(hybrid_report);
            }
        } else
            write_prismatic_mesh(input, out, hybrid ? &*hybrid : nullptr, false);
        logger().info("Wrote input + shell mesh: {}", out.string());
    }
}

void prismatic_mesh(nlohmann::json options)
{
    io::ComponentIO context;
    run(std::move(options), context);
}
ComponentResult prismatic_mesh(nlohmann::json options, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(options), context);
    return std::move(context.result);
}

} // namespace wmtk::components::prismatic_mesh
