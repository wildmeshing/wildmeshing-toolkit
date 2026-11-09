#include "prismatic_mesh.hpp"
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/DataWriter.hpp>
#include "junction_protection.hpp"
#include "rod_mesh.hpp"

#include <jse/jse.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
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
    const bool ignore_background = json_params["ignore_background_mesh"].get<bool>();
    optimization.iterations = json_params["iterations"].get<int>();
    optimization.keep_background_mesh =
        json_params["keep_background_mesh"].get<bool>() && !ignore_background;
    optimization.min_tet_volume = json_params["min_tet_volume"].get<double>();
    optimization.smoothing_max_backtracks = json_params["smoothing_max_backtracks"].get<int>();
    optimization.relative_tet_volume_floor = json_params["relative_tet_volume_floor"].get<double>();
    input.relative_tet_volume_floor = optimization.relative_tet_volume_floor;
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
    const bool rod_mode = json_params["rod_mode"].get<bool>();
    const int64_t corner_preview_vid = json_params["rod_corner_preview_vid"].get<int64_t>();
    if (corner_preview_vid >= 0 && (!rod_mode || !json_params["prism_dominant"].get<bool>()))
        log_and_throw_error("rod_corner_preview_vid requires rod_mode and prism_dominant.");
    std::optional<PrismDominantMesh> hybrid;
    if (ignore_background) discard_background_mesh(input);
    initialize_junction_protection(input);
    if (rod_mode) {
        if (corner_preview_vid >= 0 && (background.enabled || jacobian.iterations > 0))
            log_and_throw_error(
                "rod_mode uses frame construction; background_remeshing and "
                "jacobian_smoothing_iterations are surface optimizer options.");
        RodOptions rod;
        rod.radius = json_params["rod_radius"].get<double>();
        rod.thicknessratio = json_params["thicknessratio"].get<double>();
        rod.iterations = optimization.iterations;
        rod.min_tet_volume = optimization.min_tet_volume;
        rod.axial_split = json_params["rod_axial_split"].get<bool>();
        if (corner_preview_vid >= 0) {
            prepare_rod_mesh(input, rod);
            auto preview = construct_rod_corner_buffer(
                input,
                corner_preview_vid,
                json_params["rod_buffer_max_angle"].get<double>(),
                json_params["rod_buffer_max_subdivision"].get<int>());
            input = std::move(preview.input);
            hybrid = std::move(preview.hybrid);
            logger().info(
                "Rod corner preview exports an isolated buffer patch; source background is "
                "unchanged.");
        } else {
            preprocess_rod_mesh(input, rod);
            auto domain_options = optimization;
            // Output filtering preserves background validity constraints unless
            // they were explicitly disabled before preprocessing.
            domain_options.keep_background_mesh = !ignore_background;
            prism_main(input, json_params["thicknessratio"].get<double>(), domain_options);
        }
    } else
        prism_main(input, json_params["thicknessratio"].get<double>(), optimization);
    if (!hybrid && json_params["prism_dominant"].get<bool>())
        hybrid = build_prism_dominant_mesh(input, optimization.min_tet_volume);
    if (rod_mode && hybrid) {
        if (corner_preview_vid < 0 && input.rod_state->options.axial_split)
            split_rod_intervals(input, *hybrid);
        if (corner_preview_vid < 0) finalize_rod_terminal_frames(input, *hybrid);
        hybrid->report["rod"] = input.rod_state->report;
    }
    const int transition_iterations = json_params["transition_smoothing_iterations"].get<int>();
    if (transition_iterations < 0 || transition_iterations > 100)
        log_and_throw_error("transition_smoothing_iterations must be in [0,100].");
    if (transition_iterations > 0) {
        if (!hybrid) log_and_throw_error("Transition smoothing requires prism_dominant.");
        smooth_transition_regions(
            input,
            *hybrid,
            optimization.min_tet_volume,
            transition_iterations);
    }
    if (hybrid) hybrid->report["ignore_background_mesh"] = ignore_background;
    if (component_io.memory()) {
        MeshMap raw;
        nlohmann::json hybrid_report;
        write_prismatic_mesh(
            input,
            "mesh.vtu",
            hybrid ? &*hybrid : nullptr,
            !rod_mode || corner_preview_vid >= 0 || optimization.keep_background_mesh,
            &raw,
            &hybrid_report);
        component_io.result.first = io::named_outputs(std::move(raw));
        component_io.result.second = {
            {"jacobian_smoothing", input.jacobian_smoothing_report},
            {"background_remeshing", input.background_remeshing_report}};
        if (hybrid) component_io.result.second["hybrid"] = std::move(hybrid_report);
        if (rod_mode) component_io.result.second["rod"] = input.rod_state->report;
    } else {
        write_prismatic_mesh(
            input,
            output_path,
            hybrid ? &*hybrid : nullptr,
            !rod_mode || corner_preview_vid >= 0 || optimization.keep_background_mesh);
        if (rod_mode) {
            auto report_path =
                output_path.parent_path() / (output_path.stem().string() + "_rod.json");
            std::ofstream(report_path) << std::setw(2) << input.rod_state->report << '\n';
        }
    }
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
