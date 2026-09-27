#include "LaplacianSmoothing.hpp"

#include "TaggedMesh.hpp"

#include <wmtk/utils/Logger.hpp>

#include <cmath>
#include <map>

namespace wmtk::components::simwild::polyfem_helpers {

GeneratedInputs laplacian_smoothing_inputs(const TaggedMesh& mesh, const nlohmann::json& params)
{
    // Smoothing mode: the simulation JSON has no contact block, so polyfem reads no collision
    // proxy (see add_interface_constraint).
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

    GeneratedInputs out;
    const std::filesystem::path sim_in_dir = generated_dir(params, "smooth_input");
    out.sim_out_dir = generated_dir(params, "smooth_output");
    // Positions mode by default (laplacian_smoothing.run's own default, and the spec's):
    // in displacement mode the rest configuration is already a minimum and the solve
    // terminates with a zero gradient, so nothing moves.
    add_interface_constraint(
        make_interface_constraint(
            mesh,
            interfaces,
            params["use_graph_laplacian"],
            params["normalize_penalties"],
            params["scale"],
            params["smooth_positions"],
            laplacian_row_factor_by_id),
        sim_in_dir,
        /*with_collision_proxy=*/false,
        out.files);

    ReducedMesh reduced =
        reduce_mesh(operation_input(params), mesh, params["ambient_like_tags"], sim_in_dir);

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
    out.files.meshes.emplace(
        out.sim_json["geometry"][0]["mesh"].get<std::string>(),
        std::move(reduced.content));
    out.sim_json_path = sim_in_dir / "smoothing.json";
    return out;
}

OperationResult laplacian_smoothing(const TaggedMesh& mesh, const nlohmann::json& params)
{
    GeneratedInputs inputs = laplacian_smoothing_inputs(mesh, params);
    const std::unique_ptr<PolyfemBackend> backend = in_process_backend(std::move(inputs.files));

    OperationResult out;
    Eigen::MatrixXd solution;
    try {
        // One solve, no contact and no outer loop.
        solution = run_polyfem_single(
            *backend,
            inputs.sim_json,
            inputs.sim_json_path,
            inputs.sim_out_dir,
            out.solves);
    } catch (const std::exception& e) {
        throw OperationFailed(e.what(), std::move(out.solves));
    }
    // `u_mesh = u / scale`, the first of the two roundings the Python makes on the way to the
    // deformed mesh; the add is the second (write_operation_result).
    out.displacement = solution / inputs.cfg["scale"].get<double>();
    return out;
}

Eigen::MatrixXd run_polyfem_single(
    PolyfemBackend& backend,
    const OrderedJson& sim_json,
    const std::filesystem::path& sim_json_path,
    const std::filesystem::path& sim_out_dir,
    std::vector<SolveReport>& solves)
{
    const SolveResult result = backend.solve(sim_json, sim_json_path, sim_out_dir);
    solves.push_back(solve_report(sim_json, result, std::nullopt));
    check_polyfem_success(
        result.returncode,
        result.statuses,
        result.lines,
        allow_out_of_iterations(sim_json));
    return result.solution;
}

} // namespace wmtk::components::simwild::polyfem_helpers
