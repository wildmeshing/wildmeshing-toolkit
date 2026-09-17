#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::polyfem_ops {

/**
 * @brief Entry point of the polyfem-backed simwild operations (minimum separation, interface
 * smoothing), reached through the JSON-driven app and the `wildmeshing()` Python binding.
 *
 * `json_params` carries the dispatcher keys (application, input, output, operation) plus the
 * operation's own parameters, which are validated against the pysimwild spec.json of that
 * operation: those files stay the single source of the rules.
 */
void polyfem_ops(nlohmann::json json_params);

} // namespace wmtk::components::polyfem_ops
