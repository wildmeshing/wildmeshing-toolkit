#pragma once

#include "PolyfemJson.hpp"
#include "PolyfemRunner.hpp"

#include <filesystem>

namespace wmtk::components::polyfem_ops {

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

/**
 * @brief strategy="dhat": ramp dhat from the measured geometric gap until the bodies reach `sep`.
 * Mirrors `minimum_separation.step_run_polyfem` statement for statement.
 *
 * Unless the configuration pins `init_dhat`, a zero-stiffness probe solve runs first: with no
 * barrier force nothing moves, so the "active distance" polyfem reports IS the initial gap,
 * measured through the same collision proxy the real solves use. The ramp then starts at
 * growth*gap0 with the overshoot line search anchored at gap0, where the barrier exerts no force.
 * Each solve that UNDERSHOOTS commits its state (the warm start, which the backend keeps: the
 * Python renames curr_state.hdf5 over prev_state.hdf5) and sets the next dhat to growth*active;
 * each solve that OVERSHOOTS rolls back by not committing and halves the line-search step. The smallest overshooting dhat is kept as a
 * bracket upper bound and later undershoots bisect toward it instead of jumping past a dhat
 * already known to overshoot.
 *
 * `sep_json` is mutated exactly as the Python mutates its dict -- the dhat and the two state paths
 * -- and rewritten to `sep_json_path` before every solve, so what polyfem reads is the same file
 * on both engines. On return it holds the LAST attempted iteration, which is also the state the
 * Python leaves on disk.
 */
void run_polyfem_dhat(
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
 */
void run_polyfem_stiffness(
    PolyfemBackend& backend,
    OrderedJson& sep_json,
    const std::filesystem::path& sep_json_path,
    const std::filesystem::path& sim_out_dir,
    const OrderedJson& cfg);

} // namespace wmtk::components::polyfem_ops
