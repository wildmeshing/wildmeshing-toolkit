#pragma once

#include "PolyfemOperation.hpp"

#include <nlohmann/json.hpp>

namespace wmtk::components::simwild::polyfem_helpers {

/**
 * @brief The simwild operation "minimum_separation": push collision bodies apart to a target
 * separation with polyfem (AMIPS + fitting + Laplacian + GCP contact), with an outer loop per
 * `strategy`.
 *
 * `json_params` is a simwild job already verified against the simwild spec and with its defaults
 * injected, which is what `simwild()` hands over.
 * It is `prepare_minimum_separation` followed, unless `inputs_only` is set, by
 * `check_result_layout`, the outer loop on the in-process backend and the write-back of the
 * deformed mesh.
 */
void minimum_separation(nlohmann::json json_params);

/**
 * @brief Generate the simulation JSON and every input it names, up to the first solve.
 *
 * The JSON is written in every mode, and so is interface_collision.obj. In inputs_only mode every
 * other input is written too, as the Python engine wrote it; otherwise none is, and their content
 * is returned in `inputs` for the backend. Separate from `minimum_separation` so that a test can
 * build a polyfem State from exactly what a solve receives.
 */
PreparedOperation prepare_minimum_separation(nlohmann::json params);

/**
 * @brief strategy="dhat": ramp dhat from the measured geometric gap until the bodies reach `sep`.
 * Mirrors `minimum_separation.step_run_polyfem` statement for statement.
 *
 * Unless the configuration pins `init_dhat`, a zero-stiffness probe solve runs first: with no
 * barrier force nothing moves, so the "active distance" polyfem reports IS the initial gap,
 * measured through the same collision proxy the real solves use. The ramp then starts at
 * growth*gap0 with the overshoot line search anchored at gap0, where the barrier exerts no force.
 * Each solve that UNDERSHOOTS commits its state (the warm start, which the backend keeps: the
 * Python renamed curr_state.hdf5 over prev_state.hdf5) and sets the next dhat to growth*active;
 * each solve that OVERSHOOTS rolls back by not committing and halves the line-search step. The
 * smallest overshooting dhat is kept as a bracket upper bound and later undershoots bisect toward
 * it instead of jumping past a dhat already known to overshoot.
 *
 * `sep_json` is mutated exactly as the Python mutated its dict -- the dhat and the two state paths
 * -- and rewritten to `sep_json_path` before every solve, so what polyfem reads is the same file
 * on both engines. On return it holds the LAST attempted iteration, which is also the state the
 * Python left on disk.
 *
 * @return the solution of that same last solve, which is the one the Python applied to the mesh
 * (it read the solution.txt every solve overwrites): the probe's when the bodies were already
 * separated, and a rolled-back overshoot's when the allowance ran out on one.
 */
Eigen::MatrixXd run_polyfem_dhat(
    PolyfemBackend& backend,
    OrderedJson& sep_json,
    const std::filesystem::path& sep_json_path,
    const std::filesystem::path& sim_out_dir,
    const OrderedJson& cfg);

/**
 * @brief strategy="stiffness": pin dhat at sep*(1+rtol) and raise the barrier stiffness until the
 * active distance clears `sep`. Mirrors `minimum_separation.step_run_polyfem_stiffness`.
 *
 * The contact force vanishes at distances >= dhat, so the equilibrium can never exceed dhat and no
 * rollback is needed: every solve commits. The update solves the local power law
 * deficit ~ kappa^exponent for the kappa that lands at half the tolerance band, starting from the
 * theoretical exponent -1/2 and re-fitting it in log-log from the last two solves once they exist;
 * the multiplier is clamped to `max_stiffness_multiplier` per step to protect Newton conditioning.
 *
 * @return the solution of the last solve, as `run_polyfem_dhat` returns it.
 */
Eigen::MatrixXd run_polyfem_stiffness(
    PolyfemBackend& backend,
    OrderedJson& sep_json,
    const std::filesystem::path& sep_json_path,
    const std::filesystem::path& sim_out_dir,
    const OrderedJson& cfg);

} // namespace wmtk::components::simwild::polyfem_helpers
