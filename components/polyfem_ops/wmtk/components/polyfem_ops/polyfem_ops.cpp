#include "polyfem_ops.hpp"

#include <wmtk/components/simwild/polyfem_helpers/LaplacianSmoothing.hpp>
#include <wmtk/components/simwild/polyfem_helpers/MinimumSeparation.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::polyfem_ops {

namespace {

/**
 * @brief The simwild job an `{"application": "polyfem_ops"}` job now is, verified against the
 * simwild spec with its defaults injected.
 *
 * Four rewrites, each of which keeps an old job meaning what it meant:
 *  - `application` becomes "simwild", the only value the simwild spec accepts; `operation` must be
 *    one of the two polyfem operations, which this entry was always limited to.
 *  - `input` was a single path and is a list in simwild; a bare string is wrapped.
 *  - `input_dir`, which the JSON app (app/main.cpp) injects into every job, is dropped: this entry
 *    always opened `input` and `output` exactly as written, as the Python engine did, whereas
 *    simwild resolves the input against `input_dir`. Without it the spec's default "" resolves
 *    against the current directory, which is where the old entry opened it.
 *  - `max_iterations` was each operation's own key and is renamed to what simwild calls it:
 *    `max_outer_iterations` (the outer-loop cap) for minimum_separation, `nl_max_iterations`
 *    (polyfem's Newton cap) for laplacian_smoothing. In simwild `max_iterations` is the remeshing
 *    operation's key.
 */
nlohmann::json as_simwild_job(nlohmann::json json_params)
{
    const std::string operation = json_params.value("operation", "");
    if (operation != "minimum_separation" && operation != "laplacian_smoothing") {
        log_and_throw_error(
            "polyfem_ops: operation must be minimum_separation or laplacian_smoothing, got '{}'",
            operation);
    }
    json_params["application"] = "simwild";
    if (json_params.contains("input") && json_params["input"].is_string()) {
        json_params["input"] = nlohmann::json::array({json_params["input"]});
    }
    json_params.erase("input_dir");
    if (json_params.contains("max_iterations")) {
        const std::string renamed = operation == "minimum_separation" ? "max_outer_iterations"
                                                                      : "nl_max_iterations";
        if (json_params.contains(renamed)) {
            log_and_throw_error(
                "polyfem_ops: max_iterations is the old name of {} for {}; give one of the two",
                renamed,
                operation);
        }
        json_params[renamed] = json_params["max_iterations"];
        json_params.erase("max_iterations");
    }
    polyfem_helpers::validate_polyfem_operation(json_params);
    return json_params;
}

} // namespace

PreparedOperation prepare_operation(nlohmann::json json_params)
{
    json_params = as_simwild_job(std::move(json_params));
    if (json_params["operation"] == "minimum_separation") {
        return polyfem_helpers::prepare_minimum_separation(std::move(json_params));
    }
    return polyfem_helpers::prepare_laplacian_smoothing(std::move(json_params));
}

void polyfem_ops(nlohmann::json json_params)
{
    json_params = as_simwild_job(std::move(json_params));
    if (json_params["operation"] == "minimum_separation") {
        polyfem_helpers::minimum_separation(std::move(json_params));
    } else {
        polyfem_helpers::laplacian_smoothing(std::move(json_params));
    }
}

} // namespace wmtk::components::polyfem_ops
