#include "LaplacianSmoothing.hpp"

#include "TaggedMesh.hpp"

#include <wmtk/utils/Logger.hpp>

#include <cmath>
#include <map>

namespace wmtk::components::simwild::polyfem_helpers {

PreparedOperation prepare_laplacian_smoothing(nlohmann::json params)
{
    params["input"] = operation_input_path(params);

    PreparedOperation out;
    out.operation = "laplacian_smoothing";
    out.inputs_only = params["inputs_only"].get<bool>();
    const bool inputs_only = out.inputs_only;
    out.input = params["input"].get<std::string>();
    out.output = params["output"].get<std::string>();
    const std::string& input = out.input;
    const std::string& output = out.output;

    // Smoothing mode: the simulation JSON has no contact block, so polyfem reads no collision
    // proxy (see emit_interface_constraint).
    std::vector<Selection> interfaces;
    std::vector<int64_t> ids_per_input;
    assign_selection_ids(params["interfaces"], interfaces, ids_per_input);

    // The per-interface Laplacian weight, as the factor the constraint rows are scaled by.
    // polyfem gets ONE weight, `weight_laplacian` (W), for the whole Laplacian penalty, so a
    // selection that asks for its own weight w is served by scaling its nodes' rows by
    // sqrt(w / W): the row's term of W/2 ||A u - b||^2 becomes w/2 ||L_i u - b_i||^2. Only
    // the selections that carry a weight get an entry, and with none the map stays empty and
    // the constraint is not touched at all.
    const double weight_laplacian = params["weight_laplacian"].get<double>();
    std::map<int64_t, double> laplacian_row_factor_by_id;
    for (const auto& selection : interfaces) {
        if (!selection.weight.has_value()) {
            continue;
        }
        if (weight_laplacian <= 0.0) {
            // The factor is relative to W, so W = 0 (the Laplacian penalty switched off) has
            // no factor that expresses w: the two keys contradict each other.
            log_and_throw_error(
                "interface region='{}' asks for weight {} while weight_laplacian is {}: a "
                "per-interface weight is relative to weight_laplacian, which must be "
                "positive",
                selection.region,
                *selection.weight,
                weight_laplacian);
        }
        laplacian_row_factor_by_id[*selection.id] =
            std::sqrt(*selection.weight / weight_laplacian);
    }

    const std::filesystem::path sim_in_dir = sim_dir(output, "smooth_input");
    out.sim_out_dir = sim_dir(output, "smooth_output");
    logger().info("Input  : {}", input);
    logger().info("Output : {}.msh", output);
    // Positions mode by default (laplacian_smoothing.run's own default, and the spec's):
    // in displacement mode the rest configuration is already a minimum and the solve
    // terminates with a zero gradient, so nothing moves.
    emit_interface_constraint(
        make_interface_constraint(
            input,
            interfaces,
            params["use_graph_laplacian"],
            params["normalize_penalties"],
            params["scale"],
            params["smooth_positions"],
            laplacian_row_factor_by_id),
        sim_in_dir,
        /*with_collision_proxy=*/false,
        inputs_only,
        out.inputs);

    ReducedMesh reduced =
        reduce_mesh(input, params["ambient_like_tags"], sim_in_dir, inputs_only);

    // The reduced mesh's two groups are "ambient" and "body", which is the scheme
    // build_polyfem_json looks weights up in, so the resolved pair replaces whatever the
    // configuration carried.
    out.cfg = laplacian_smoothing_cfg(params);
    out.cfg["amips_weights"] = resolve_amips_weights(out.cfg);
    out.sim_json = build_polyfem_json(
        out.cfg,
        reduced.path,
        sim_in_dir,
        reduced.info,
        out.sim_out_dir / "solution.txt");
    if (!inputs_only) {
        out.inputs.meshes.emplace(
            out.sim_json["geometry"][0]["mesh"].get<std::string>(),
            std::move(reduced.content));
    }
    out.sim_json_path = sim_in_dir / "smoothing.json";
    write_polyfem_json(out.sim_json_path, out.sim_json);
    return out;
}

void laplacian_smoothing(nlohmann::json json_params)
{
    PreparedOperation prepared = prepare_laplacian_smoothing(std::move(json_params));
    if (prepared.inputs_only) {
        return;
    }
    const std::unique_ptr<PolyfemBackend> backend = operation_backend(prepared);

    // One solve, no contact and no outer loop: `step_run_polyfem_single` writes the same
    // JSON again itself, which is mirrored rather than skipped so the two engines touch the
    // file the same number of times.
    run_polyfem_single(
        *backend,
        prepared.sim_json,
        prepared.sim_json_path,
        prepared.sim_out_dir);

    write_operation_result(prepared);
}

void run_polyfem_single(
    PolyfemBackend& backend,
    const OrderedJson& sim_json,
    const std::filesystem::path& sim_json_path,
    const std::filesystem::path& sim_out_dir)
{
    std::filesystem::create_directories(sim_out_dir);
    write_polyfem_json(sim_json_path, sim_json);

    const SolveResult result =
        backend.solve(sim_json_path, sim_out_dir, sim_out_dir / "polyfem.log");
    check_polyfem_success(
        result.returncode,
        result.statuses,
        result.lines,
        allow_out_of_iterations(sim_json));
}

} // namespace wmtk::components::simwild::polyfem_helpers
