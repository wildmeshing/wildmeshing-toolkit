#include "prismatic_mesh.hpp"
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/DataWriter.hpp>
#include "junction_protection.hpp"
#include "quality_improvement.hpp"
#include "rod_mesh.hpp"
#include "thickness_smoothing.hpp"

#include <jse/jse.h>
#include <algorithm>
#include <cmath>
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
    const bool quality_improvement = json_params["quality_improvement"].get<bool>();
    QualityImprovementOptions quality_options;
    if (quality_improvement) {
        if (!json_params["prism_dominant"].get<bool>())
            log_and_throw_error("Quality improvement requires prism_dominant.");
        if (json_params["min_tet_volume"].get<double>() != 0 ||
            json_params["relative_tet_volume_floor"].get<double>() != 0)
            log_and_throw_error(
                "Region quality improvement requires min_tet_volume=0 and "
                "relative_tet_volume_floor=0; it enforces strictly positive actual cells.");
        quality_options =
            QualityImprovementOptions::from_json(json_params["quality_improvement_options"]);
        // Region remeshing starts at the original hybrid build. Its mixed
        // smoothing replaces the separate optional quality stages.
        for (const char* name :
             {"jacobian_smoothing_iterations",
              "transition_smoothing_iterations",
              "thickness_smoothing_iterations"})
            json_params[name] = 0;
        json_params["background_remeshing"] = false;
    }
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
    if (json_params["rod_edge_based"].get<bool>() && !rod_mode)
        log_and_throw_error("rod_edge_based requires rod_mode.");
    if (json_params["rod_background_remeshing"].get<bool>() &&
        (!rod_mode || !json_params["rod_edge_based"].get<bool>() || ignore_background))
        log_and_throw_error("rod_background_remeshing requires edge-based rods and background.");
    if (json_params["rod_coupled_optimization_rounds"].get<int>() > 0 &&
        (!rod_mode || !json_params["rod_edge_based"].get<bool>() || ignore_background ||
         !json_params["rod_background_remeshing"].get<bool>()))
        log_and_throw_error(
            "rod_coupled_optimization_rounds requires edge-based rods and local background "
            "remeshing.");
    std::optional<PrismDominantMesh> hybrid;
    std::optional<ImprovedHybridMesh> improved, shell_improved;
    if (ignore_background) discard_background_mesh(input);
    initialize_junction_protection(input);
    if (rod_mode) {
        RodOptions rod;
        rod.edge_based = json_params["rod_edge_based"].get<bool>();
        rod.radius = json_params["rod_radius"].get<double>();
        rod.thicknessratio = json_params["thicknessratio"].get<double>();
        rod.iterations = optimization.iterations;
        rod.position_iterations = json_params["rod_position_iterations"].get<int>();
        rod.position_weight = json_params["rod_position_weight"].get<double>();
        rod.laplace_weight = json_params["rod_laplace_weight"].get<double>();
        rod.position_smoothing_weight = json_params["rod_position_smoothing_weight"].get<double>();
        rod.position_smoothing_iterations =
            json_params["rod_position_smoothing_iterations"].get<int>();
        rod.position_smoothing_geometry_weight =
            json_params["rod_position_smoothing_geometry_weight"].get<double>();
        rod.joint_smoothing_iterations = json_params["rod_joint_smoothing_iterations"].get<int>();
        rod.joint_smoothing_weight = json_params["rod_joint_smoothing_weight"].get<double>();
        rod.joint_geometry_weight = json_params["rod_joint_geometry_weight"].get<double>();
        rod.joint_blend_radius_ratio = json_params["rod_joint_blend_radius_ratio"].get<double>();
        rod.coupled_rounds = json_params["rod_coupled_optimization_rounds"].get<int>();
        rod.coupled_iterations = json_params["rod_coupled_optimization_iterations"].get<int>();
        rod.coupled_smoothing_weight = json_params["rod_coupled_smoothing_weight"].get<double>();
        rod.coupled_geometry_weight = json_params["rod_coupled_geometry_weight"].get<double>();
        rod.min_tet_volume = optimization.min_tet_volume;
        rod.background_remeshing.enabled = json_params["rod_background_remeshing"].get<bool>();
        rod.background_remeshing.radius =
            json_params["rod_background_remeshing_radius"].get<double>();
        rod.background_remeshing.target_length =
            json_params["rod_background_remeshing_target_length"].get<double>();
        rod.background_remeshing.passes = json_params["rod_background_remeshing_passes"].get<int>();
        rod.background_remeshing.time_limit =
            json_params["rod_background_remeshing_time_limit"].get<double>();
        preprocess_rod_mesh(input, rod);
        auto domain_options = optimization;
        domain_options.keep_background_mesh = !ignore_background;
        prism_main(input, rod.thicknessratio, domain_options);
        nlohmann::json shell_report;
        const bool mixed = has_shell_region(input) && json_params["prism_dominant"].get<bool>();
        if (mixed) {
            hybrid = build_shell_prism_mesh(input, optimization.min_tet_volume);
            shell_report = hybrid->report;
            if (quality_improvement) {
                auto candidate = improve_prismatic_mesh_quality(input, *hybrid, quality_options);
                try {
                    auto replacement = adopt_improved_shell_mesh(input, candidate);
                    hybrid = std::move(replacement);
                    shell_report["quality_improvement"] = candidate.report;
                    shell_improved = std::move(candidate);
                } catch (const std::exception& e) {
                    shell_report["quality_improvement"] = {
                        {"applied", false},
                        {"reason", e.what()}};
                    logger().warn(
                        "Shell remeshing checkpoint cannot be handed to rods: {}",
                        e.what());
                }
            }
            reserve_shell_cells(input, *hybrid);
        }
        if (rod.edge_based) optimize_rod_tet_positions(input);
        if (rod.edge_based) optimize_rod_joint_positions(input);
        if (rod.edge_based) {
            if (rod.coupled_rounds > 0)
                optimize_rod_background_coupled(input);
            else
                remesh_rod_background(input);
        }
        if (mixed) {
            hybrid = build_rod_prism_mesh(input, optimization.min_tet_volume);
            hybrid->report["shell_reconstruction"] = std::move(shell_report);
            hybrid->report["pipeline_order"] = {
                "shell_construction",
                "shell_quality",
                "rod_geometry",
                "rod_construction"};
        }
    } else
        prism_main(input, json_params["thicknessratio"].get<double>(), optimization);
    if (!hybrid && json_params["prism_dominant"].get<bool>())
        hybrid = build_prism_dominant_mesh(input, optimization.min_tet_volume);
    if (rod_mode && hybrid) {
        finalize_rod_ends(input, *hybrid);
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
    const int thickness_iterations = json_params["thickness_smoothing_iterations"].get<int>();
    if (thickness_iterations < 0 || thickness_iterations > 200)
        log_and_throw_error("thickness_smoothing_iterations must be in [0,200].");
    if (thickness_iterations > 0) {
        if (!hybrid) log_and_throw_error("Thickness smoothing requires prism_dominant.");
        smooth_thickness_and_prisms(
            input,
            *hybrid,
            optimization.min_tet_volume,
            thickness_iterations);
    }
    if (hybrid) hybrid->report["ignore_background_mesh"] = ignore_background;
    if (quality_improvement) {
        improved = rod_mode ? finalize_staged_mixed_mesh(
                                  input,
                                  *hybrid,
                                  shell_improved ? &*shell_improved : nullptr,
                                  optimization.keep_background_mesh)
                            : improve_prismatic_mesh_quality(input, *hybrid, quality_options);
        improved->report["construction"] = hybrid->report;
    } else if (rod_mode && hybrid && optimization.keep_background_mesh) {
        auto candidate = finalize_staged_mixed_mesh(input, *hybrid, nullptr, true);
        const auto& background = candidate.report["background_relocation"];
        hybrid->report["background_relocation"] = background;
        hybrid->report["final_native_quality"] = candidate.report["final_quality"];
        hybrid->report["final_domain_volume"] = candidate.report["final_domain_volume"];
        hybrid->report["final_material_topology"] = candidate.report["final_material_topology"];
        if (background.value("accepted", size_t(0)) > 0) improved = std::move(candidate);
    }
    const auto write_result = [&](const std::filesystem::path& destination,
                                  bool background,
                                  MeshMap* raw = nullptr,
                                  nlohmann::json* report = nullptr) {
        if (improved)
            write_improved_prismatic_mesh(*improved, destination, background, raw, report);
        else
            write_prismatic_mesh(
                input,
                destination,
                hybrid ? &*hybrid : nullptr,
                background,
                raw,
                report);
    };
    if (component_io.memory()) {
        MeshMap raw;
        nlohmann::json hybrid_report;
        write_result(
            "mesh.vtu",
            !rod_mode || optimization.keep_background_mesh,
            &raw,
            &hybrid_report);
        component_io.result.first = io::named_outputs(std::move(raw));
        component_io.result.second = {
            {"jacobian_smoothing", input.jacobian_smoothing_report},
            {"background_remeshing", input.background_remeshing_report}};
        if (hybrid) component_io.result.second["hybrid"] = std::move(hybrid_report);
        if (rod_mode) component_io.result.second["rod"] = input.rod_state->report;
    } else {
        write_result(output_path, !rod_mode || optimization.keep_background_mesh);
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
                write_result("mesh.vtu", false, &raw, &hybrid_report);
                auto keys = io::default_output_keys();
                keys["mesh.vtu"] = "input_and_shell";
                keys["mesh_offset_faces.vtu"] = "input_and_shell_offset";
                component_io.result.first.merge(io::named_outputs(std::move(raw), keys));
                if (hybrid)
                    component_io.result.second["input_and_shell_hybrid"] = std::move(hybrid_report);
            }
        } else
            write_result(out, false);
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
