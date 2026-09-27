#pragma once

#include "PolyfemOperation.hpp"
#include "TaggedMesh.hpp"

#include <nlohmann/json.hpp>

namespace wmtk::components::simwild::polyfem_helpers {

/**
 * @brief laplacian_smoothing on `mesh`, in memory: fair material interfaces with one polyfem solve
 * (AMIPS + fitting + Laplacian, no contact).
 *
 * `params` is a simwild job already verified against the simwild spec and with its defaults
 * injected, which is what `simwild()` hands over. Its `input` and `output` only name the generated
 * files (`laplacian_smoothing_inputs`), and `inputs_only` is not read.
 *
 * It is `laplacian_smoothing_inputs` followed by the single solve on the in-process backend. No
 * file is read or written -- under `save_vtu`, polyfem's paraview frames excepted
 * (PolyfemBackend) -- and `mesh` is not changed.
 *
 * @throws OperationFailed when the solve failed, carrying its report
 */
OperationResult laplacian_smoothing(const TaggedMesh& mesh, const nlohmann::json& params);

/**
 * @brief The simulation JSON and every input it names, generated from `mesh` up to the solve: what
 * inputs_only writes, as the Python engine wrote it, and what the solve hands polyfem.
 *
 * The files are named in `generated_dir(params, "smooth_input")` and polyfem's output in
 * `generated_dir(params, "smooth_output")`; the reduced mesh is named after the input file.
 * Nothing is read or written.
 */
GeneratedInputs laplacian_smoothing_inputs(const TaggedMesh& mesh, const nlohmann::json& params);

/**
 * @brief One polyfem solve, no contact and no outer loop. Mirrors
 * `polyfem_utils.step_run_polyfem_single`, which is the whole of the smoothing engine's solve.
 *
 * The Python's first argument is the path of the binary; this takes the backend instead, which is
 * the one place the two engines are allowed to differ -- see PolyfemRunner.hpp on why the boundary
 * exists. The solve's report is appended to `solves` before it is checked.
 *
 * @return the solve's solution.
 */
Eigen::MatrixXd run_polyfem_single(
    PolyfemBackend& backend,
    const OrderedJson& sim_json,
    const std::filesystem::path& sim_json_path,
    const std::filesystem::path& sim_out_dir,
    std::vector<SolveReport>& solves);

} // namespace wmtk::components::simwild::polyfem_helpers
