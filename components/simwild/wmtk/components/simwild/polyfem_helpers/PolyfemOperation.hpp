#pragma once

#include "InterfaceSelection.hpp"
#include "PolyfemJson.hpp"
#include "PolyfemRunner.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <memory>
#include <string>

namespace wmtk::components::simwild::polyfem_helpers {

/// An operation up to its first solve: everything its preparation generates and the solve
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
 * @brief Validate `json_params` against `simwild::simwild_spec_for(json_params)` in strict mode and
 * inject
 * the defaults, in place.
 *
 * What `wmtk::utils::verify_and_setup_logger` does for the simwild entry, without the logger, for
 * the polyfem_ops entry that forwards to the same operations.
 */
void validate_polyfem_operation(nlohmann::json& json_params);

/// The single input mesh of a polyfem operation: `json_params["input"]` resolved against
/// `json_params["input_dir"]`, as every other simwild operation resolves it. These operations solve
/// on one already-tagged .msh, so a list of several is a configuration error.
std::string operation_input_path(const nlohmann::json& json_params);

/**
 * @brief One of the directories the generated polyfem inputs and outputs go in: `sep_input` /
 * `sep_output` or `smooth_input` / `smooth_output` beside the output stem. Mirrors the Python
 * engine's `out_dir = os.path.dirname(p["output"]) or "."` followed by
 * minimum_separation.run / laplacian_smoothing.run.
 *
 * Canonical, because both operations resolve the directory before use and the resolved strings go
 * verbatim into the simulation JSON: on this machine /tmp and /var are symlinks, so an unresolved
 * path would name the same directory by a different name than the Python did.
 */
std::filesystem::path sim_dir(const std::string& output, const std::string& subdir);

/// The reduced mesh and its material groups, which is what `build_polyfem_json` is given.
struct ReducedMesh
{
    std::filesystem::path path;
    mshio::MshSpec content;
    MeshInfo info;
};

/**
 * @brief The mesh polyfem actually solves on, and its material groups. Both operations do this,
 * in this order, for the same reason.
 *
 * The volumetric solve never runs on the caller's multi-tag mesh: WMTK writes one copy of a
 * multi-tagged cell per tag, and polyfem reads the copies as distinct elements -- it double-counts
 * AMIPS in separation and segfaults during constraint setup in smoothing. Collision filtering is
 * unaffected either way, because it works on the proxy mesh and not on body ids.
 *
 * The file is `<input stem>_polyfem.msh` in the simulation input directory, where the Python engine
 * put it next to the other generated inputs. inputs_only writes it and reads the groups back off
 * it, as the Python did. A normal run writes no file and reads the groups off the content, in the
 * same order: the per-group volume is a running sum that divides every AMIPS weight in the
 * simulation JSON, so this is what keeps that JSON the same byte for byte in both modes (checked
 * in tests/test_polyfem_in_process.cpp).
 */
ReducedMesh reduce_mesh(
    const std::string& input,
    const std::vector<std::string>& ambient_like_tags,
    const std::filesystem::path& sim_in_dir,
    bool inputs_only);

/**
 * @brief The files of `make_interface_constraint`, under the names `build_polyfem_json` gives
 * them in `dir`: written in inputs_only mode, kept in `memory` under those names otherwise.
 *
 * `with_collision_proxy` is false in smoothing mode, whose simulation JSON has no contact block:
 * polyfem reads neither the linear map nor the body ids there, and the Python did not write them.
 */
void emit_interface_constraint(
    const InterfaceConstraint& generated,
    const std::filesystem::path& dir,
    bool with_collision_proxy,
    bool inputs_only,
    SolveInputs& memory);

/// `curr_json.get("solver", {}).get("nonlinear", {}).get("allow_out_of_iterations", False)`.
bool allow_out_of_iterations(const OrderedJson& doc);

/// The polyfem linked into this process, holding `prepared`'s generated inputs in memory (which it
/// takes over). The Python engine ran $POLYFEM_BIN instead; this one needs no binary and never
/// looks at the variable.
std::unique_ptr<PolyfemBackend> operation_backend(PreparedOperation& prepared);

/**
 * @brief Write the solved mesh to `<output>.msh`, which is the last thing both operations do.
 *
 * Applied to the ORIGINAL mesh, which is what preserves the caller's full tag set on the output;
 * the node tags match between the original and the reduced mesh, so solution.txt indexes
 * consistently against either.
 */
void write_operation_result(const PreparedOperation& prepared);

} // namespace wmtk::components::simwild::polyfem_helpers
