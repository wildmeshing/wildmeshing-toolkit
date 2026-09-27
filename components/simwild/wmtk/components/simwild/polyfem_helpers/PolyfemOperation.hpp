#pragma once

#include "InterfaceSelection.hpp"
#include "PolyfemJson.hpp"
#include "PolyfemRunner.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace wmtk::components::simwild::polyfem_helpers {

/**
 * @brief Everything an operation generates before its first solve: the content of every file
 * inputs_only writes, and the configuration the solve steers by. Nothing in it was read from or
 * written to a file.
 *
 * The paths are names only. They are the files inputs_only writes, and the simulation JSON names
 * its inputs by them, so the solve finds each input's content under the name the JSON gives it.
 */
struct GeneratedInputs
{
    OrderedJson cfg; ///< the engine configuration the outer loops steer by
    OrderedJson sim_json; ///< the simulation JSON, as inputs_only writes it to `sim_json_path`
    std::filesystem::path sim_json_path;
    /// The directory the simulation JSON puts polyfem's output in: the solution file and the
    /// warm-start states, which the backend keeps in memory, and under `save_vtu` the paraview
    /// frames, which polyfem writes there.
    std::filesystem::path sim_out_dir;
    SolveInputs files; ///< every other file inputs_only writes
};

/// One polyfem solve of an operation: the document it was handed, and what came back
/// (`SolveResult`).
struct SolveReport
{
    /// The outer-loop iteration of the solve; empty for the probe of the dhat ramp and for the
    /// one solve of smoothing.
    std::optional<int64_t> iteration;
    /// The simulation JSON as the solve handed it to polyfem: the document the Python engine
    /// wrote to disk before that solve.
    OrderedJson sim_json;
    std::optional<double> active_distance;
    std::vector<polysolve::nonlinear::Status> statuses;
    std::vector<std::string> log; ///< polyfem's own output, line by line
};

/// What an operation returns.
struct OperationResult
{
    /// The last solve's solution divided by the configuration's `scale`: one row per node of the
    /// mesh (its TaggedMesh node id), one column per dimension, in mesh units.
    Eigen::MatrixXd displacement;
    std::vector<SolveReport> solves; ///< in the order they ran
};

/// A solve that failed, or an outer loop that stopped on an error after its first solve: the
/// error's message, and the report of every solve that ran, the failed one last, so that the
/// logs of a failed run are not lost with it.
class OperationFailed : public std::runtime_error
{
public:
    OperationFailed(const std::string& what, std::vector<SolveReport> solves_)
        : std::runtime_error(what)
        , solves(std::move(solves_))
    {}

    std::vector<SolveReport> solves;
};

/**
 * @brief The one input file of an operation, `params["input"]` as given. These operations solve
 * on one already-tagged mesh, so a list of several is a configuration error.
 *
 * The file is not opened here: the operation is handed the mesh, and names the generated files
 * after this one (`reduce_mesh`).
 */
std::string operation_input(const nlohmann::json& params);

/**
 * @brief One of the directories the generated files are named in: `sep_input` / `sep_output` or
 * `smooth_input` / `smooth_output` beside the output stem, `params["output"]`. Mirrors the Python
 * engine's `out_dir = os.path.dirname(p["output"]) or "."` followed by minimum_separation.run /
 * laplacian_smoothing.run.
 *
 * A name only, taken from `output` as it stands: nothing is created or resolved. The Python
 * resolved the directory, and the resolved strings went verbatim into the simulation JSON; the
 * simwild operation (polyfem_operations.cpp) resolves `output` before it gets here, which keeps
 * that JSON the Python's byte for byte.
 */
std::filesystem::path generated_dir(const nlohmann::json& params, const std::string& subdir);

/// The reduced mesh and its material groups, which is what `build_polyfem_json` is given.
struct ReducedMesh
{
    std::filesystem::path path;
    ReducedMsh content;
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
 * `mesh` is the caller's mesh, `input` the name of its file. The reduced mesh is named
 * `<input stem>_polyfem.msh` in the simulation input directory, where the Python engine wrote it
 * next to the other generated inputs. The groups are read off the content, in the order the file
 * lists them, where the Python read them back off the file: the per-group volume is a running sum
 * that divides every AMIPS weight in the simulation JSON, so this is what keeps that JSON the
 * same byte for byte (checked in tests/test_polyfem_in_process.cpp).
 */
ReducedMesh reduce_mesh(
    const std::string& input,
    const TaggedMesh& mesh,
    const std::vector<std::string>& ambient_like_tags,
    const std::filesystem::path& sim_in_dir);

/**
 * @brief The files of `make_interface_constraint`, put into `files` under the names
 * `build_polyfem_json` gives them in `dir`.
 *
 * `with_collision_proxy` is false in smoothing mode, whose simulation JSON has no contact block:
 * polyfem reads neither the linear map nor the body ids there, and the Python did not write them.
 * The OBJ is put in in both modes: the Python engine always wrote it, and it is the only
 * convenient way to see which faces the selection picked.
 */
void add_interface_constraint(
    const InterfaceConstraint& generated,
    const std::filesystem::path& dir,
    bool with_collision_proxy,
    SolveInputs& files);

/// `curr_json.get("solver", {}).get("nonlinear", {}).get("allow_out_of_iterations", False)`.
bool allow_out_of_iterations(const OrderedJson& doc);

/// The report of the solve of `sim_json`, `iteration` as `SolveReport::iteration`.
SolveReport solve_report(
    const OrderedJson& sim_json,
    const SolveResult& result,
    std::optional<int64_t> iteration);

} // namespace wmtk::components::simwild::polyfem_helpers
