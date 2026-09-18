#include "PolyfemRunner.hpp"

#include <wmtk/utils/Logger.hpp>

#include <polyfem/State.hpp>
#include <polyfem/solver/forms/SmoothContactForm.hpp>
#include <polyfem/time_integrator/ImplicitTimeIntegrator.hpp>
#include <polyfem/utils/Logger.hpp>

#include <ipc/utils/logger.hpp>

#include <spdlog/sinks/base_sink.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <mutex>
#include <optional>

namespace wmtk::components::polyfem_ops {

namespace {

/**
 * @brief The warm start between two solves, in memory: exactly the three matrices polyfem's
 * `ImplicitTimeIntegrator::save_state` writes into `curr_state.hdf5` and
 * `State::initial_solution` / `initial_velocity` / `initial_acceleration` read back out of
 * `prev_state.hdf5`.
 *
 * The round trip through hdf5 is lossless -- `write_matrix`/`read_matrix` store float64 -- so
 * carrying the matrices instead of the file changes no number; what it removes is a file write and
 * a file read per outer iteration.
 */
struct SolverState
{
    Eigen::MatrixXd solution; ///< "u": the time integrator's x_prevs(), one column per stored step
    Eigen::MatrixXd velocity; ///< "v": v_prevs()
    Eigen::MatrixXd acceleration; ///< "a": a_prevs()
};

/**
 * @brief The active distance polyfem's contact form measures at `sol`, or nothing when polyfem
 * would not have logged one.
 *
 * `polyfem::solver::SmoothContactForm::post_step` is the only place the
 * "Minimum distance during solve: ..., active distance: ..., dhat: ..." line comes from, and it
 * computes exactly these two calls on `collision_set()` -- the set cached at the last solution the
 * solver visited, which is `sol` -- and prints nothing at all when the minimum distance is
 * infinite. Reproducing that guard is what makes "no line in the log" and "no value here" the same
 * event for the loops, which stop on it with "contact not triggered".
 */
std::optional<double> active_distance_from_contact_form(
    const polyfem::State& state,
    const Eigen::MatrixXd& sol);

/**
 * @brief The sink that turns polyfem's own log records into `polyfem_iter_<i>.log` (and into the
 * lines whose last "Finished:" line `check_polyfem_success` quotes when a solve failed).
 *
 * The Python engine copies the executable's stdout into that file. In process there is no child
 * stdout to copy, so the text is taken where it is produced: spdlog's default pattern formats each
 * record exactly as the `stdout_color_sink` the executable installs formats it, and the escapes
 * polyfem embeds in a few messages with `fmt::fg` (the "timing" tag) are stripped here just as the
 * Python strips them out of its capture.
 *
 * `stop()` exists because `State::init` REPLACES polyfem's global logger on every solve: a logger
 * this sink was attached to can outlive the solve if something still holds it, and after stop()
 * such a straggler writes nothing instead of writing into a closed file.
 */
class LogCapture : public spdlog::sinks::base_sink<std::mutex>
{
public:
    explicit LogCapture(const std::filesystem::path& log_path)
        : m_file(log_path)
    {
        if (!m_file.is_open()) {
            log_and_throw_error("Unable to open {} for writing", log_path.string());
        }
    }

    void stop()
    {
        std::lock_guard<std::mutex> lock(base_sink<std::mutex>::mutex_);
        m_stopped = true;
        m_file.close();
    }

    /// The captured records, split on '\n' exactly as the Python splits the executable's output.
    std::vector<std::string> lines()
    {
        std::lock_guard<std::mutex> lock(base_sink<std::mutex>::mutex_);
        return m_lines;
    }

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        if (m_stopped) {
            return;
        }
        spdlog::memory_buf_t formatted;
        base_sink<std::mutex>::formatter_->format(msg, formatted);
        const std::string text = strip_ansi(fmt::to_string(formatted));
        m_file << text;
        for (auto& line : split_lines(text)) {
            m_lines.push_back(std::move(line));
        }
    }

    void flush_() override
    {
        if (!m_stopped) {
            m_file.flush();
        }
    }

private:
    std::ofstream m_file;
    std::vector<std::string> m_lines;
    bool m_stopped = false;
};

/// polyfem's default log level (json-specs/log.json: /output/log/level defaults to "debug"). The
/// executable only overrides it when `--log_level` is passed and the Python engine never passes
/// it, so this is the level both engines' log files are written at. Read off the document anyway,
/// so that a JSON that did set it would still produce matching logs.
spdlog::level::level_enum document_log_level(const nlohmann::json& doc)
{
    const auto output = doc.find("output");
    if (output != doc.end() && output->is_object()) {
        const auto log = output->find("log");
        if (log != output->end() && log->is_object()) {
            const auto level = log->find("level");
            if (level != log->end() && level->is_string()) {
                return spdlog::level::from_str(level->get<std::string>());
            }
        }
    }
    return spdlog::level::debug;
}

/// Put `sink` on `logger` unless it is already there. polyfem's logger, ipc's logger and (through
/// GeogramUtils) geogram's output are the three things the executable prints on stdout; they are
/// two distinct logger objects sharing one set of sinks, so both have to be fed.
void attach_sink(spdlog::logger& logger, const spdlog::sink_ptr& sink)
{
    auto& sinks = logger.sinks();
    if (std::find(sinks.begin(), sinks.end(), sink) == sinks.end()) {
        sinks.push_back(sink);
    }
}

void detach_sink(spdlog::logger& logger, const spdlog::sink_ptr& sink)
{
    auto& sinks = logger.sinks();
    sinks.erase(std::remove(sinks.begin(), sinks.end(), sink), sinks.end());
}

/// `main.cpp`'s `load_json`: the document, with `root_path` defaulted to the file it came from so
/// that relative paths inside it resolve the same way.
nlohmann::json load_simulation_json(const std::filesystem::path& json_path)
{
    std::ifstream file(json_path);
    if (!file.is_open()) {
        log_and_throw_error("unable to open {} file", json_path.string());
    }
    nlohmann::json out;
    file >> out;
    if (!out.contains("root_path")) {
        out["root_path"] = json_path.string();
    }
    return out;
}

std::optional<double> active_distance_from_contact_form(
    const polyfem::State& state,
    const Eigen::MatrixXd& sol)
{
    const auto* form = dynamic_cast<const polyfem::solver::SmoothContactForm*>(
        state.solve_data.contact_form.get());
    if (form == nullptr) {
        // No contact at all (the smoothing operation), or a formulation whose post_step does not
        // log the line: either way the executable's log has no line for the Python to parse.
        return std::nullopt;
    }
    const Eigen::MatrixXd displaced = form->compute_displaced_surface(sol.col(0));
    const double minimum =
        form->collision_set().compute_minimum_distance(state.collision_mesh, displaced);
    if (std::isinf(minimum)) {
        // SmoothContactForm::post_step's own guard: with nothing within dhat it prints no line.
        return std::nullopt;
    }
    // post_step prints sqrt() of the squared distance the collision set reports.
    return std::sqrt(
        form->collision_set().compute_active_minimum_distance(state.collision_mesh, displaced));
}

/**
 * @brief The in-process backend: `src/polyfem/main.cpp`'s `forward_simulation`, call for call, on
 * a State built here.
 *
 * Three inputs still go through files because polyfem has no in-memory entry point for them and
 * this step adds none: the soft constraints, the hard pins and the collision proxy with its linear
 * map. The reduced mesh stays a file too -- see the comment on `run_solve` -- so the simulation
 * JSON is byte for byte the one the Python engine hands the executable, and polyfem's own readers
 * open all four.
 *
 * What does NOT go through a file any more is the warm start: the JSON's `input/data/state` and
 * `output/data/state` are blanked in the in-memory copy of the arguments, and the three matrices
 * polyfem would have written to `curr_state.hdf5` are carried in `m_last` instead.
 */
class InProcessBackend : public PolyfemBackend
{
public:
    SolveResult solve(
        const std::filesystem::path& json_path,
        const std::filesystem::path& out_dir,
        const std::filesystem::path& log_path) override
    {
        nlohmann::json args = load_simulation_json(json_path);

        // The loop puts prev_state.hdf5 into the document exactly when it has committed a solve,
        // which is exactly when m_committed holds one; the file itself is never written or read.
        const bool wants_warm_start =
            args.contains("input") && args["input"].contains("data") &&
            args["input"]["data"].contains("state") &&
            !args["input"]["data"]["state"].get<std::string>().empty();
        if (wants_warm_start && !m_committed.has_value()) {
            log_and_throw_error(
                "{} asks polyfem to warm start from {}, but no solve has been committed in this "
                "process",
                json_path.string(),
                args["input"]["data"]["state"].get<std::string>());
        }
        if (args.contains("input") && args["input"].contains("data")) {
            args["input"]["data"]["state"] = "";
        }
        if (args.contains("output") && args["output"].contains("data")) {
            args["output"]["data"]["state"] = "";
        }

        // `-o <out_dir>`, the one command-line argument the Python engine passes the executable
        // besides `-j`, applied the way main.cpp applies it.
        nlohmann::json patch = nlohmann::json::object();
        patch["/output/directory"_json_pointer] = std::filesystem::absolute(out_dir).string();
        args.merge_patch(patch);

        if (!log_path.parent_path().empty()) {
            std::filesystem::create_directories(log_path.parent_path());
        }
        auto capture = std::make_shared<LogCapture>(log_path);
        capture->set_level(document_log_level(args));
        // Attached BEFORE State::init, which logs (the linear-solver choice, among others) before
        // it installs its own logger, and again after, because init replaces the logger object.
        attach_sink(polyfem::logger(), capture);
        attach_sink(ipc::logger(), capture);

        SolveResult result;
        try {
            result = run_solve(args, capture, wants_warm_start);
        } catch (const std::exception& e) {
            // The executable dies on this (an uncaught exception aborts main, so the Python engine
            // sees return code -6); in process there is no signal to report, so the failure is a
            // non-zero code and check_polyfem_success prints its banner as usual.
            // polyfem has already logged the message itself -- log_and_throw_error logs before it
            // throws -- so it is in the captured lines and in the log file.
            logger().error("polyfem failed in process: {}", e.what());
            result.returncode = 1;
            m_last.reset();
        }

        detach_sink(polyfem::logger(), capture);
        detach_sink(ipc::logger(), capture);
        capture->flush();
        result.lines = capture->lines();
        capture->stop();
        return result;
    }

    void reset_warm_start() override
    {
        // Nothing on disk to unlink, only the two solutions to drop.
        m_last.reset();
        m_committed.reset();
    }

    void commit_warm_start() override { m_committed = m_last; }

    bool has_warm_start() const override { return m_committed.has_value(); }

private:
    /// main.cpp's `forward_simulation` for a JSON input, with the initial condition handed over in
    /// memory. Returns the active distance and the subsolve statuses, both read off the State
    /// before it is destroyed; `returncode` and `lines` are the caller's.
    SolveResult run_solve(
        const nlohmann::json& args,
        const std::shared_ptr<LogCapture>& capture,
        const bool wants_warm_start)
    {
        polyfem::State state;
        state.init(args, /*strict_validation=*/true);
        attach_sink(polyfem::logger(), capture);
        attach_sink(ipc::logger(), capture);

        // The reduced mesh stays a file. `load_mesh(V, F)` would skip `read_fem_geometry`, and
        // with it `Mesh::create(path)`, which is what reads the .msh physical tags into the mesh's
        // body ids -- the ids every material in this JSON is keyed by. The geometry block's
        // `scale` is not the obstacle (it is an exactly reproducible per-coordinate product; see
        // tests/test_polyfem_in_process.cpp, which measures both).
        state.load_mesh(/*non_conforming=*/false, {}, {}, {});
        if (state.mesh == nullptr) {
            // main.cpp returns EXIT_FAILURE here; load_mesh has already logged why.
            throw std::runtime_error("unable to load the mesh");
        }
        state.stats.compute_mesh_stats(*state.mesh);

        state.build_basis();

        state.assemble_rhs();
        state.assemble_mass_mat();

        Eigen::MatrixXd sol;
        Eigen::MatrixXd pressure;

        polyfem::InitialConditionOverride initial_condition;
        if (wants_warm_start) {
            initial_condition.solution = m_committed->solution;
            initial_condition.velocity = m_committed->velocity;
            initial_condition.acceleration = m_committed->acceleration;
        }
        state.solve_problem(sol, pressure, {}, wants_warm_start ? &initial_condition : nullptr);

        state.compute_errors(sol);

        polyfem::logger().info("total time: {}s", state.timings.total_time());

        state.save_json(sol);
        state.export_data(sol, pressure);

        // What polyfem would have written to curr_state.hdf5 at this point: the transient loop
        // calls save_state after update_quantities, so x_prevs() already holds this solve's
        // solution.
        m_last.reset();
        if (state.solve_data.time_integrator != nullptr) {
            const auto& integrator = *state.solve_data.time_integrator;
            const int ndof = int(integrator.x_prev().size());
            const int prev_steps = int(integrator.x_prevs().size());
            SolverState next;
            next.solution.resize(ndof, prev_steps);
            next.velocity.resize(ndof, prev_steps);
            next.acceleration.resize(ndof, prev_steps);
            for (int i = 0; i < prev_steps; ++i) {
                next.solution.col(i) = integrator.x_prevs()[i];
                next.velocity.col(i) = integrator.v_prevs()[i];
                next.acceleration.col(i) = integrator.a_prevs()[i];
            }
            m_last = std::move(next);
        }

        SolveResult result;
        result.active_distance = active_distance_from_contact_form(state, sol);
        // One entry per AL, reduced and lagging subsolve; an entry lacks a status only when no
        // solver ran (polyfem's ALSolver::record_solver_info).
        for (const auto& entry : state.stats.solver_info) {
            const auto& info = entry.at("info");
            if (info.contains("status")) {
                result.statuses.push_back(info.at("status").get<polysolve::nonlinear::Status>());
            }
        }
        return result;
    }

    std::optional<SolverState> m_last; ///< polyfem's curr_state.hdf5
    std::optional<SolverState> m_committed; ///< polyfem's prev_state.hdf5
};

} // namespace

std::unique_ptr<PolyfemBackend> in_process_backend()
{
    return std::make_unique<InProcessBackend>();
}

} // namespace wmtk::components::polyfem_ops
