#pragma once

#include "PolyfemOperation.hpp"

#include <nlohmann/json.hpp>

namespace wmtk::components::simwild::polyfem_helpers {

/**
 * @brief The simwild operation "laplacian_smoothing": fair material interfaces with one polyfem
 * solve (AMIPS + fitting + Laplacian, no contact).
 *
 * `json_params` is a simwild job already verified against the simwild spec and with its defaults
 * injected, which is what `simwild()` hands over.
 * It is `prepare_laplacian_smoothing` followed, unless `inputs_only` is set, by the single solve on
 * the in-process backend and the write-back of the deformed mesh.
 */
void laplacian_smoothing(nlohmann::json json_params);

/**
 * @brief Generate the simulation JSON and every input it names, up to the solve.
 *
 * The JSON is written in every mode, and so is interface_collision.obj. In inputs_only mode every
 * other input is written too, as the Python engine wrote it; otherwise none is, and their content
 * is returned in `inputs` for the backend. Separate from `laplacian_smoothing` so that a test can
 * build a polyfem State from exactly what a solve receives.
 */
PreparedOperation prepare_laplacian_smoothing(nlohmann::json params);

/**
 * @brief One polyfem solve, no contact and no outer loop. Mirrors
 * `polyfem_utils.step_run_polyfem_single`, which is the whole of the smoothing engine's solve.
 *
 * Writes `sim_json` to `sim_json_path` and logs to `sim_out_dir/polyfem.log`. The Python's first
 * argument is the path of the binary; this takes the backend instead, which is the one place the
 * two engines are allowed to differ -- see PolyfemRunner.hpp on why the boundary exists.
 */
void run_polyfem_single(
    PolyfemBackend& backend,
    const OrderedJson& sim_json,
    const std::filesystem::path& sim_json_path,
    const std::filesystem::path& sim_out_dir);

} // namespace wmtk::components::simwild::polyfem_helpers
