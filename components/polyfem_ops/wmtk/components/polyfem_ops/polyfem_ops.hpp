#pragma once

#include "PolyfemJson.hpp"
#include "PolyfemRunner.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <string>

namespace wmtk::components::polyfem_ops {

/**
 * @brief Entry point of the polyfem-backed simwild operations (minimum separation, interface
 * smoothing), reached through the JSON-driven app and the `wildmeshing()` Python binding.
 *
 * `json_params` carries the dispatcher keys (application, input, output, operation) plus the
 * operation's own parameters, which are validated against the pysimwild spec.json of that
 * operation: those files stay the single source of the rules.
 *
 * It is `prepare_operation` followed, unless `inputs_only` is set, by the solve on the in-process
 * backend and the write-back of the deformed mesh.
 */
void polyfem_ops(nlohmann::json json_params);

/// An operation up to its first solve: everything `prepare_operation` generates and the solve
/// consumes.
struct PreparedOperation
{
    std::string operation;
    bool inputs_only = false;
    std::string input; ///< the caller's multi-tag .msh
    std::string output; ///< the output stem; the deformed mesh goes to <output>.msh
    OrderedJson cfg; ///< the engine configuration the outer loops steer by
    OrderedJson sim_json; ///< the simulation JSON, as written to `sim_json_path`
    std::filesystem::path sim_json_path;
    std::filesystem::path sim_out_dir;
    /// The content of every input file `sim_json` names. Empty in inputs_only mode, which writes
    /// those files instead.
    SolveInputs inputs;
};

/**
 * @brief Validate `json_params` and generate the simulation JSON and every input it names.
 *
 * The JSON is written in every mode, and so is interface_collision.obj. In inputs_only mode every
 * other input is written too, as the Python engine writes it; otherwise none is, and their content
 * is returned in `inputs` for the backend. Separate from `polyfem_ops` so that a test can build a
 * polyfem State from exactly what a solve receives.
 */
PreparedOperation prepare_operation(nlohmann::json json_params);

} // namespace wmtk::components::polyfem_ops
