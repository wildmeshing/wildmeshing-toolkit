#pragma once

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <string>

namespace wmtk::components::simwild {

/**
 * @brief The simwild operation "minimum_separation" on files: the input .msh read into a
 * TaggedMesh, the in-memory operation (`polyfem_helpers::minimum_separation`) run on it, and what
 * it returns written. This is the only code of the operation that touches a file.
 *
 * `json_params` is a simwild job already verified against the simwild spec and with its defaults
 * injected, which is what `simwild()` hands over.
 *
 * With `inputs_only`, the generated polyfem inputs are written to `<directory of output>/sep_input`
 * as the Python engine wrote them, and nothing is solved. Otherwise the input is first checked to
 * be a file `write_operation_result` can write the result into (`check_result_layout`); the run
 * then writes each solve's polyfem log into `<directory of output>/sep_output` and the deformed
 * mesh to `<output>.msh`, and nothing else -- under `save_vtu`, polyfem's paraview frames in
 * sep_output excepted, and under `write_simulation_json` the last solve's simulation JSON, to
 * sep_input/separation.json.
 */
void minimum_separation(const nlohmann::json& json_params);

/// The simwild operation "laplacian_smoothing" on files, as `minimum_separation` is:
/// `polyfem_helpers::laplacian_smoothing` on the input, with smooth_input and smooth_output in
/// place of sep_input and sep_output.
void laplacian_smoothing(const nlohmann::json& json_params);

/**
 * @brief Throw unless `write_operation_result` can write a result for `input`: the file has to be
 * in the layout wmtk::MshData writes, which is checked by rebuilding it with nothing moved and
 * comparing (`require_rebuildable` in polyfem_operations.cpp). Both operations call it before their
 * first solve, so an input the result could not be written into is refused before any time is
 * spent on it.
 */
void check_result_layout(const std::string& input);

/**
 * @brief Write the solved mesh to `<output>.msh`, which is the last thing both operations do.
 * Mirrors the arithmetic of `polyfem_utils.step_write_deformed_msh`.
 *
 * The result is the ORIGINAL mesh of the file `input` -- every physical group, including one of a
 * lower dimension such as simwild's "EnvelopeSurface", in file order, with its cells in file order
 * and its element and node tags -- with only the mesh nodes moved. It is rebuilt with
 * wmtk::MshData (`load`, then per group `get_VF` and the matching `add_*`), which reproduces
 * exactly the files written in its own layout (`check_result_layout`) and nothing else, and saved
 * binary, which stores every coordinate exactly.
 *
 * `displacement` is `OperationResult::displacement`, one row per node of the file, in mesh units:
 * the Python's `u_mesh = u / scale`. The rows of the mesh nodes (the first group's) are added, as
 * the Python adds them, so both engines round at the same two places; the nodes of a
 * lower-dimensional group keep their positions.
 */
void write_operation_result(
    const std::string& input,
    const std::string& output,
    const Eigen::MatrixXd& displacement);

} // namespace wmtk::components::simwild
