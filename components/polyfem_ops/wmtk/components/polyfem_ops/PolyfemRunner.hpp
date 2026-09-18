#pragma once

#include <polysolve/nonlinear/Criteria.hpp>

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace wmtk::components::polyfem_ops {

/**
 * @brief The PolyFEM binary named by $POLYFEM_BIN. Mirrors `polyfem_utils.polyfem_bin`, the two
 * error messages included.
 *
 * The environment variable is the single supported way to point the operations at a build, on
 * both engines: no search path, no configuration key. Only the subprocess backend needs it; the
 * in-process one is linked against polyfem and never looks at it.
 */
std::string polyfem_bin();

/// What one solve reports back. `returncode` and `lines` are what `polyfem_utils.run_streaming`
/// returns -- the exit code and the output lines, each still carrying its trailing newline, as the
/// Python's `lines` do -- and `active_distance` is the value the dhat and stiffness loops steer
/// on, filled by the backend rather than parsed by the caller (see `PolyfemBackend::solve`).
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
 * @brief The one and only way the outer loops reach polyfem: a simulation JSON goes in, the
 * solver's output and its active distance come back, and a copy of that output is left in
 * `log_path`.
 *
 * Two implementations sit behind it -- a child process running $POLYFEM_BIN and a polyfem linked
 * into this one -- and nothing above this boundary may know which is in use. That is why the warm
 * start is here too: the subprocess backend can only pass it through the two hdf5 files the
 * simulation JSON names, while the in-process one carries a solution vector from one solve to the
 * next, and the loops must write the same JSON and take the same decisions either way.
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

    /**
     * @brief Forget every warm start, and say where the two state files live.
     *
     * Called once before an outer loop starts and once after it ends -- the two places the Python
     * unlinks `curr_state.hdf5` and `prev_state.hdf5`. The subprocess backend needs the paths (its
     * child writes and reads those files through the JSON); the in-process one ignores them.
     */
    virtual void reset_warm_start(
        const std::filesystem::path& curr_state,
        const std::filesystem::path& prev_state) = 0;

    /// Accept the last solve as the warm start for the next one. This is the Python's
    /// `prev_state.hdf5 <- curr_state.hdf5` rename, and a solve that is NOT committed is exactly
    /// the rollback the dhat ramp performs on an overshoot.
    virtual void commit_warm_start() = 0;

    /// Whether a committed warm start exists. The loops put `input/data/state` into the simulation
    /// JSON exactly when it does, so the document on disk is the same on both backends.
    virtual bool has_warm_start() const = 0;
};

/**
 * @brief The subprocess backend: runs `<binary> -j <json_path> -o <out_dir>`, streams the output
 * to this process's stdout as it arrives and tees an ANSI-stripped copy to `log_path`. Mirrors
 * `polyfem_utils.run_streaming`.
 *
 * The Python runs the child under a pseudo-terminal, which exists only so spdlog keeps its colours
 * on the human's screen; a plain pipe is used here. The log file is the part that has to agree,
 * and it agrees because the Python strips the escapes back out before writing it -- a pipe simply
 * never has them, since spdlog's colour sink checks for a terminal. The two logs still differ in
 * their line endings: a pty translates '\n' to '\r\n' on the way out and the Python's ANSI filter
 * does not remove the '\r', so its logs carry one per line and these do not.
 *
 * Its warm start is the pair of hdf5 files: polyfem writes `curr_state.hdf5` because the JSON
 * names it as `output/data/state` and reads `prev_state.hdf5` back as `input/data/state`.
 */
std::unique_ptr<PolyfemBackend> subprocess_backend(const std::string& binary);

/**
 * @brief The in-process backend: the same call sequence `src/polyfem/main.cpp` performs, on a
 * `polyfem::State` built in this process from the same JSON file.
 *
 * Defined in PolyfemInProcess.cpp so that polyfem's headers stay out of this one. What it takes
 * from memory instead of from a file or a log is documented there.
 */
std::unique_ptr<PolyfemBackend> in_process_backend();

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

/**
 * @brief The statuses `check_polyfem_success` accepts, recovered from logged "Finished:" lines --
 * how the SUBPROCESS backend fills `SolveResult::statuses`, since the child reports them only in
 * its log. A line counts when it contains "Finished: " followed by polysolve's `status_message`
 * of an accepted status, which is the phrase the Python searches for.
 */
std::vector<polysolve::nonlinear::Status> statuses_from_log(const std::vector<std::string>& lines);

/**
 * @brief The active distance polyfem reported, or nothing when it never did. Mirrors the two
 * identical parses in `minimum_separation.step_run_polyfem` and `step_run_polyfem_stiffness`.
 *
 * This is how the SUBPROCESS backend fills `SolveResult::active_distance`; the in-process one
 * asks the contact form instead and only this test-visible function still reads the text.
 *
 * The value is read off the LAST line that contains "active distance:" (polyfem logs
 * "Minimum distance during solve: <d>, active distance: <a>, dhat: <h>" after every Newton step,
 * so the last one is the converged state), taking the text after the last occurrence of the
 * marker in that line, its first whitespace-separated token, stripping trailing ',' and ';' and
 * removing every remaining ',', then converting with `strtod` -- Python's `float()` on the same
 * token, not on a reformatted one, so no digit is lost on the way.
 *
 * "inf" parses as infinity on both sides, which the loops test for explicitly.
 */
std::optional<double> parse_active_distance(const std::vector<std::string>& lines);

/// `polyfem_utils._ANSI_RE` applied to a whole chunk of text. A pipe never carries an escape from
/// spdlog's colour sink -- it asks whether stdout is a terminal -- but polyfem writes a few of its
/// own with `fmt::fg` INSIDE the message (the "timing" tag), so both backends have escapes to
/// remove before writing the log file, and the Python strips them out of its pty capture too.
std::string strip_ansi(const std::string& text);

/// Split captured output the way `run_streaming`'s pty branch does: on '\n', with the newline put
/// back on every piece and a trailing empty piece dropped. The lines are what
/// `check_polyfem_success` and `parse_active_distance` read, so the split has to be the same one
/// on both backends.
std::vector<std::string> split_lines(const std::string& text);

} // namespace wmtk::components::polyfem_ops
