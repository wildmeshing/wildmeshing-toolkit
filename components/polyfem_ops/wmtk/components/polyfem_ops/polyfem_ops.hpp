#pragma once

#include <wmtk/components/simwild/polyfem_helpers/PolyfemOperation.hpp>

#include <nlohmann/json.hpp>

namespace wmtk::components::polyfem_ops {

/// The polyfem-facing code this component forwards to; it lives in the simwild component
/// (components/simwild/wmtk/components/simwild/polyfem_helpers).
namespace polyfem_helpers = wmtk::components::simwild::polyfem_helpers;

/**
 * @brief The old JSON entry of the polyfem-backed operations (minimum separation, interface
 * smoothing), kept as a forwarder while both entries are compared; the operations are now simwild's
 * (`{"application": "simwild", "operation": ...}`).
 *
 * `json_params` is rewritten into the simwild job it now is (see as_simwild_job in
 * polyfem_ops.cpp), validated against the simwild spec, and handed to the same operation function
 * simwild calls.
 */
void polyfem_ops(nlohmann::json json_params);

using PreparedOperation = polyfem_helpers::PreparedOperation;

/**
 * @brief The operation's preparation up to its first solve, through the same rewrite and
 * validation as `polyfem_ops`: `prepare_minimum_separation` or `prepare_laplacian_smoothing`.
 * Kept so that a test can build a polyfem State from exactly what a solve receives.
 */
PreparedOperation prepare_operation(nlohmann::json json_params);

} // namespace wmtk::components::polyfem_ops
