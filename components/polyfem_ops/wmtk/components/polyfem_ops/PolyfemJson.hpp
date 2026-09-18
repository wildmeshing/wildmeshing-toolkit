#pragma once

#include "MeshReduction.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <string>

namespace wmtk::components::polyfem_ops {

/// Every JSON built here is an `ordered_json`: nlohmann's default object sorts its keys, while
/// `json.dumps` writes a Python dict in INSERTION order, and the simulation JSON is compared key
/// order and all. The engine configuration is one too, so that the order the Python builds its
/// `cfg` dict in is visible in the code that mirrors it.
using OrderedJson = nlohmann::ordered_json;

/**
 * @brief `polyfem_utils.OPT_DEFAULTS`: the engine-level defaults `build_polyfem_json` falls back
 * to for a key the configuration does not carry.
 *
 * The spec-derived half is read out of the EMBEDDED minimum_separation spec.json at run time,
 * exactly as `_minsep_spec_defaults()` reads the file: root-level rules (one '/' in the pointer)
 * that declare a default. Both engines share this one table, the smoothing engine included --
 * `polyfem_utils` builds it once from the separation spec. Keeping a second copy here is the trap
 * the Python's own comment names (the max_iterations 5-vs-10 one), so there is none.
 */
const OrderedJson& opt_defaults();

/// Recursive dict merge: `override` wins, nested objects merge. Mirrors `polyfem_utils.deep_merge`.
OrderedJson deep_merge(const OrderedJson& base, const OrderedJson& override_value);

/**
 * @brief One polyfem geometry entry: mesh plus a `{"scale": scale}` transformation. Mirrors
 * `polyfem_utils.geometry_block`.
 *
 * The Python's `transformation` and `surface_selection` arguments are not mirrored: no caller
 * passes either, so the deep merge is always with an empty override and no surface selection is
 * ever attached.
 */
OrderedJson geometry_block(const std::string& mesh_path, const OrderedJson& scale);

/**
 * @brief Resolve the configuration's AMIPS weights to the reduced mesh's two-body scheme. Mirrors
 * `polyfem_utils._resolve_amips_weights`.
 *
 * Returns `{"ambient": w, "body": w}`: the dedicated `amips_ambient_weight`/`amips_body_weight`
 * keys win, then the `amips_weights` dict (its "ambient" entry, and its FIRST non-ambient value),
 * then the engine defaults.
 *
 * "First non-ambient value" is where the two engines can disagree, and the difference is not
 * repairable here: the Python reads a dict in insertion order, while this component is handed an
 * already-parsed `nlohmann::json` object whose keys are sorted, the caller's order having been
 * lost before the operation was entered. With one non-ambient key -- every case the spec
 * documents -- the two agree; with several, the Python takes the caller's first and this takes the
 * alphabetically first.
 */
OrderedJson resolve_amips_weights(const OrderedJson& cfg);

/**
 * @brief Build the polyfem simulation JSON. Mirrors `polyfem_utils.build_polyfem_json`.
 *
 * Per-tag AMIPS (or NeoHookean) materials, the soft fitting/Laplacian constraints from `out_dir`,
 * an optional contact block, the solver settings, quasistatic time, an all-fixed Dirichlet
 * condition and the output paths. `msh_path` must be the REDUCED two-body mesh and `info` must be
 * `get_mesh_info` on it: the Python takes that function's five return values as five arguments,
 * and they are passed here as the one struct it returns.
 */
OrderedJson build_polyfem_json(
    const OrderedJson& cfg,
    const std::filesystem::path& msh_path,
    const std::filesystem::path& out_dir,
    const MeshInfo& info,
    const std::filesystem::path& sol_path);

/**
 * @brief The engine configuration `simwild.minimum_separation` builds from the validated spec
 * parameters, with the mutations `minimum_separation.run` makes to it before the solve.
 *
 * The renames are the Python's (`use_fitting` -> `useFitting`, and so on), and so is the mapping
 * of `use_nh_body`, `nh_youngs` and `nh_poisson`: the Python wrapper used to declare those three
 * in its spec and then leave them out of the configuration, so the bodies stayed AMIPS whatever
 * the caller asked for. Both engines now pass them through.
 */
OrderedJson minimum_separation_cfg(const nlohmann::json& params, const OrderedJson& polyfem_pairs);

/**
 * @brief The engine configuration `simwild.laplacian_smoothing` builds, with the mutations
 * `laplacian_smoothing.run` makes: contact forced off, `max_iterations` aliased to the polyfem
 * nonlinear cap (there is no outer loop here, so the name is unambiguous), the body AMIPS weight
 * defaulted to 1e-4 (with volume-normalized AMIPS the weights share the Laplacian's currency and
 * smoothing wants the fairing to win) and positions mode on by default.
 */
OrderedJson laplacian_smoothing_cfg(const nlohmann::json& params);

/// Write the simulation JSON the way `json.dumps(doc, indent=4)` + `Path.write_text` does: four
/// spaces of indent and NO trailing newline.
void write_polyfem_json(const std::filesystem::path& path, const OrderedJson& doc);

} // namespace wmtk::components::polyfem_ops
