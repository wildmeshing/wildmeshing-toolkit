#pragma once

#include "Hdf5Writers.hpp"
#include "InterfaceSelection.hpp"
#include "MeshReduction.hpp"

#include <polysolve/nonlinear/Criteria.hpp>

#include <spdlog/common.h>
#include <nlohmann/json.hpp>

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace polyfem {
class State;
}

namespace wmtk::components::polyfem_ops {

/// What one solve reports back. `returncode` and `lines` stand for what
/// `polyfem_utils.run_streaming` returns -- the exit code and the output lines, each still carrying
/// its trailing newline, as the Python's `lines` do -- and `active_distance` is the value the dhat
/// and stiffness loops steer on, taken from polyfem rather than parsed out of its log (see
/// `PolyfemBackend::solve`).
/// `statuses` is polysolve's termination status of every nonlinear subsolve that returned, in
/// order; it is what `check_polyfem_success` decides on.
struct SolveResult
{
    int returncode = 0;
    std::vector<std::string> lines;
    std::optional<double> active_distance;
    std::vector<polysolve::nonlinear::Status> statuses;
};

/**
 * @brief The content of every input file an operation's simulation JSON names, keyed by the path
 * the JSON names it by: `geometry[0].mesh` in `meshes`, each `constraints.hard[*]` and
 * `constraints.soft[*].data` in `constraints`, and `contact.collision_mesh.{mesh, linear_map,
 * collision_body_ids}` in the last three.
 *
 * Built once per operation, before its first solve, from the same arrays inputs_only writes to
 * those paths. The outer loops never change it: between solves they rewrite `contact.dhat`, the
 * barrier stiffness and the two warm-start paths, none of which names an input.
 */
struct SolveInputs
{
    std::map<std::string, mshio::MshSpec> meshes;
    std::map<std::string, ConstraintHdf5> constraints;
    std::map<std::string, CollisionObj> collision_meshes;
    std::map<std::string, LinearMapHdf5> linear_maps;
    std::map<std::string, std::vector<std::vector<int64_t>>> collision_body_ids;
};

/**
 * @brief The one and only way the outer loops reach polyfem: a simulation JSON goes in, the
 * solver's output and its active distance come back, and a copy of that output is left in
 * `log_path`.
 *
 * The JSON is still the single description of what a solve reads, and the document on disk stays
 * the one the Python engine writes, file names included. What the backend does not do is read
 * those files: it holds their content (`SolveInputs`) and the warm start in memory, and hands both
 * to polyfem from there (see PolyfemInProcess.cpp).
 */
class PolyfemBackend
{
public:
    virtual ~PolyfemBackend() = default;

    /**
     * @brief One solve. `json_path` is the simulation JSON, `out_dir` is what the executable takes
     * as `-o`, and `log_path` receives polyfem's own output.
     *
     * `active_distance` is set when polyfem reported one and left empty when it did not, which is
     * how the loops tell "contact not triggered" from a measurement.
     */
    virtual SolveResult solve(
        const std::filesystem::path& json_path,
        const std::filesystem::path& out_dir,
        const std::filesystem::path& log_path) = 0;

    /// Forget every warm start. Called once before an outer loop starts and once after it ends --
    /// the two places the Python unlinks `curr_state.hdf5` and `prev_state.hdf5`.
    virtual void reset_warm_start() = 0;

    /// Accept the last solve as the warm start for the next one. This is the Python's
    /// `prev_state.hdf5 <- curr_state.hdf5` rename, and a solve that is NOT committed is exactly
    /// the rollback the dhat ramp performs on an overshoot.
    virtual void commit_warm_start() = 0;

    /// Whether a committed warm start exists. The loops put `input/data/state` into the simulation
    /// JSON exactly when it does, which is when the Python's `prev_state.hdf5` would exist.
    virtual bool has_warm_start() const = 0;
};

/**
 * @brief The in-process backend: the same call sequence `src/polyfem/main.cpp` performs, on a
 * `polyfem::State` built in this process from the same JSON file and from `inputs`, which must
 * hold the content of every input file that JSON names.
 *
 * Defined in PolyfemInProcess.cpp so that polyfem's headers stay out of this one. What it takes
 * from memory instead of from a file or a log is documented there.
 */
std::unique_ptr<PolyfemBackend> in_process_backend(SolveInputs inputs);

/**
 * @brief Everything the in-process backend does to a State before it solves: `State::init` on
 * `args`, the inputs the JSON names taken from `inputs` (a name with no content there throws; the
 * files are never read), the mesh, the basis and the assembly.
 *
 * `log_sink` is attached to polyfem's and ipc's loggers right after `State::init`, which replaces
 * both; null attaches nothing. Declared here, and not only used inside the backend, so that a test
 * can build a State exactly the way a solve does and compare it with one polyfem builds from the
 * files.
 */
void prepare_state(
    polyfem::State& state,
    const nlohmann::json& args,
    const SolveInputs& inputs,
    const spdlog::sink_ptr& log_sink);

/**
 * @brief Throw unless polyfem converged. Mirrors `polyfem_utils.check_polyfem_success`, including
 * the banner it prints before throwing and the text of the exception.
 *
 * The rule is the Python's: the exit code is 0 and some subsolve ended on the absolute or the
 * relative gradient tolerance, or, with `allow_out_of_iterations`, on the iteration limit. The
 * Python reads that off the "Finished: <status message>" lines polysolve logs when a subsolve
 * returns; this reads the same statuses from `statuses`. `lines` only supply the last
 * "Finished:" line the failure message quotes.
 */
void check_polyfem_success(
    int returncode,
    const std::vector<polysolve::nonlinear::Status>& statuses,
    const std::vector<std::string>& lines,
    bool allow_out_of_iterations);

/// `polyfem_utils._ANSI_RE` applied to a whole chunk of text. polyfem writes a few escapes of its
/// own with `fmt::fg` INSIDE the message (the "timing" tag), so they have to be removed before the
/// log file is written, as the Python strips them out of its pty capture.
std::string strip_ansi(const std::string& text);

/// Split captured output the way `run_streaming`'s pty branch does: on '\n', with the newline put
/// back on every piece and a trailing empty piece dropped.
std::vector<std::string> split_lines(const std::string& text);

} // namespace wmtk::components::polyfem_ops
