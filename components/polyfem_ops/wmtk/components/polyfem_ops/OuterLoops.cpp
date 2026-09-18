#include "OuterLoops.hpp"

#include "NumpyCompat.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace wmtk::components::polyfem_ops {

namespace {

constexpr double INF = std::numeric_limits<double>::infinity();

/// `dict.get(key, fallback)` for a number: the stored value when the key is there, the fallback
/// when it is not.
double get_number(const OrderedJson& cfg, const std::string& key, const double fallback)
{
    const auto it = cfg.find(key);
    return it == cfg.end() ? fallback : it->get<double>();
}

/// `cfg.get("max_iterations", OPT_DEFAULTS["max_iterations"])`, read as the integer the Python's
/// `range()` takes.
int64_t max_iterations(const OrderedJson& cfg)
{
    const auto it = cfg.find("max_iterations");
    return it == cfg.end() ? opt_defaults().at("max_iterations").get<int64_t>()
                           : it->get<int64_t>();
}

/// `curr_json.get("solver", {}).get("nonlinear", {}).get("allow_out_of_iterations", False)`.
bool allow_out_of_iterations(const OrderedJson& doc)
{
    const auto solver = doc.find("solver");
    if (solver == doc.end() || !solver->is_object()) return false;
    const auto nonlinear = solver->find("nonlinear");
    if (nonlinear == solver->end() || !nonlinear->is_object()) return false;
    const auto flag = nonlinear->find("allow_out_of_iterations");
    return flag != nonlinear->end() && flag->get<bool>();
}

/// `for stale in sim_out_dir.glob("polyfem_*.log"): stale.unlink()` -- a shorter rerun must not
/// leave the iteration logs of a longer one behind, which would be read as this run's.
void remove_stale_iteration_logs(const std::filesystem::path& sim_out_dir)
{
    for (const auto& entry : std::filesystem::directory_iterator(sim_out_dir)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("polyfem_", 0) == 0 && name.size() > 12 &&
            name.compare(name.size() - 4, 4, ".log") == 0) {
            std::filesystem::remove(entry.path());
        }
    }
}

/// The Python's loop variable survives the `for` and is read afterwards; when `range()` was empty
/// there is no such variable and the read is a NameError. Mirrored as a throw, with the same
/// meaning: a run with no iterations at all is a configuration error.
void require_one_iteration(const int64_t n)
{
    if (n <= 0) {
        log_and_throw_error(
            "max_iterations is {}, so no solve would run; the Python engine raises NameError on "
            "its loop variable here",
            n);
    }
}

/// The two hdf5 files the executable carries a warm start through: it writes `curr` because the
/// document names it as `output/data/state` and reads `prev` back as `input/data/state`. The loops
/// still name them so the document stays the Python engine's; the backend blanks both and keeps
/// the states in memory.
struct WarmStartPaths
{
    std::filesystem::path curr;
    std::filesystem::path prev;
};

/// The warm-start block both loops open with, in the order both Python loops write it: name the
/// two state files, drop any warm start left over -- the Python unlinks the files here, since
/// otherwise the loop would warm start from a previous run's last accepted solve and silently
/// begin at the wrong initial configuration -- and point `output/data/state` at the one polyfem
/// writes.
WarmStartPaths open_warm_start(
    PolyfemBackend& backend,
    OrderedJson& curr_json,
    const std::filesystem::path& sim_out_dir)
{
    const WarmStartPaths paths{sim_out_dir / "curr_state.hdf5", sim_out_dir / "prev_state.hdf5"};
    backend.reset_warm_start();
    curr_json["output"]["data"]["state"] = paths.curr.string();
    return paths;
}

/**
 * @brief One solve of an outer loop, and nothing either loop decides.
 *
 * The committed warm start goes into the document exactly when there is one (so the file on disk
 * is the one the Python engine writes), the document is written to `sep_json_path`, polyfem runs
 * with its own per-iteration log, and its output is checked. Both Python loops do these four steps
 * identically; what they do with the answer is where they part.
 *
 * @return the active distance polyfem reported, or nothing when it reported none -- the loops stop
 * on that, and the message they both print for it is here so there is one copy of it.
 */
std::optional<double> solve_iteration(
    PolyfemBackend& backend,
    OrderedJson& curr_json,
    const std::filesystem::path& sep_json_path,
    const std::filesystem::path& sim_out_dir,
    const WarmStartPaths& state,
    const int64_t iter)
{
    if (backend.has_warm_start()) {
        curr_json["input"]["data"]["state"] = state.prev.string();
    }
    write_polyfem_json(sep_json_path, curr_json);

    const SolveResult result = backend.solve(
        sep_json_path,
        sim_out_dir,
        sim_out_dir / fmt::format("polyfem_iter_{}.log", iter));
    check_polyfem_success(
        result.returncode,
        result.statuses,
        result.lines,
        allow_out_of_iterations(curr_json));

    if (!result.active_distance.has_value()) {
        logger().info("No active distance found in output — contact not triggered. Stopping.");
    }
    return result.active_distance;
}

/// The message both loops end on when they used up their allowance. Read after the loop, so it
/// fires on a `break` at the last index too: a run that reached its target on its final allowance
/// still prints it. Measured on the boxes3d stiffness case, which succeeds at iteration 3 of 4 and
/// prints it.
void log_if_out_of_iterations(
    const int64_t last_iter,
    const int64_t n_iterations,
    const double active_dist)
{
    if (last_iter == n_iterations - 1) {
        logger().info(
            "Reached maximum iterations ({}) without achieving desired separation. Final active "
            "distance: {:.6e}",
            last_iter + 1,
            active_dist);
    }
}

} // namespace

void run_polyfem_single(
    PolyfemBackend& backend,
    const OrderedJson& sim_json,
    const std::filesystem::path& sim_json_path,
    const std::filesystem::path& sim_out_dir)
{
    std::filesystem::create_directories(sim_out_dir);
    write_polyfem_json(sim_json_path, sim_json);

    const SolveResult result =
        backend.solve(sim_json_path, sim_out_dir, sim_out_dir / "polyfem.log");
    check_polyfem_success(
        result.returncode,
        result.statuses,
        result.lines,
        allow_out_of_iterations(sim_json));
}

void run_polyfem_dhat(
    PolyfemBackend& backend,
    OrderedJson& sep_json,
    const std::filesystem::path& sep_json_path,
    const std::filesystem::path& sim_out_dir,
    const OrderedJson& cfg)
{
    std::filesystem::create_directories(sim_out_dir);
    remove_stale_iteration_logs(sim_out_dir);

    double active_dist = -INF;
    const double sep = cfg.at("sep").get<double>();
    const double growth =
        get_number(cfg, "dhat_growth", opt_defaults().at("dhat_growth").get<double>());
    const bool has_init_dhat = cfg.contains("init_dhat");
    const double init_dhat = has_init_dhat ? cfg.at("init_dhat").get<double>() : sep;

    sep_json["contact"]["dhat"] = init_dhat;

    // The Python's `curr_json = sep_json.copy()` is SHALLOW, so its writes land in `sep_json` too;
    // one mutable document gives the same sequence of files on disk.
    OrderedJson& curr_json = sep_json;
    const WarmStartPaths state = open_warm_start(backend, curr_json, sim_out_dir);
    double committed_dhat = init_dhat;
    double alpha = 1.0;
    std::optional<double> line_search_step;
    std::optional<double> prev_active;
    bool stall_warned = false;
    std::optional<double> dhat_high; // smallest dhat known to overshoot (bracket upper bound)

    if (!has_init_dhat) {
        // Zero-stiffness probe: no contact force, so nothing moves and the reported "active
        // distance" IS the initial gap, measured through the same collision proxy the real solves
        // use. Seed the ramp at growth*gap0 and anchor the overshoot line search at gap0, where
        // the barrier exerts no force, so a first-step overshoot halves back toward the measured
        // gap instead of tripping the assert below.
        OrderedJson probe_json = sep_json;
        probe_json["solver"]["contact"]["barrier_stiffness"] = 0.0;
        write_polyfem_json(sep_json_path, probe_json);
        const SolveResult probe =
            backend.solve(sep_json_path, sim_out_dir, sim_out_dir / "polyfem_probe.log");
        check_polyfem_success(
            probe.returncode,
            probe.statuses,
            probe.lines,
            /*allow_out_of_iterations=*/true);
        const std::optional<double> gap_line = probe.active_distance;
        const double gap0 = gap_line.value_or(INF);
        if (!(gap0 < sep)) {
            logger().info(
                "Probe: initial gap {} >= sep {:.6e} — already separated. Stopping.",
                gap_line.has_value() ? fmt::format("{:.6e}", gap0) : "not within dhat",
                sep);
            backend.reset_warm_start();
            return;
        }
        logger().info("Probe: initial gap {:.6e}", gap0);
        committed_dhat = gap0;
        line_search_step = (growth - 1.0) * gap0;
        prev_active = gap0;
        curr_json["contact"]["dhat"] = growth * gap0;
        logger().info(
            "Starting dhat ramp at {:.6e}",
            curr_json["contact"]["dhat"].get<double>());
    }

    const int64_t n_iterations = max_iterations(cfg);
    require_one_iteration(n_iterations);
    // The Python's `for iter in range(...)` leaves its loop variable behind and reads it after
    // the loop; `last_iter` is that variable, and nothing else.
    int64_t last_iter = -1;
    for (int64_t iter = 0; iter < n_iterations; ++iter) {
        last_iter = iter;
        const std::optional<double> parsed =
            solve_iteration(backend, curr_json, sep_json_path, sim_out_dir, state, iter);
        if (!parsed.has_value()) {
            break;
        }
        active_dist = *parsed;

        logger().info("Current active distance: {:.6e}", active_dist);

        if (active_dist == INF) {
            logger().info("Active distance is infinite — no contact. Stopping.");
            break;
        }
        const double rtol = get_number(cfg, "rtol", opt_defaults().at("rtol").get<double>());
        if (numpy_isclose(active_dist, sep, rtol) && active_dist > sep) {
            logger().info(
                "Desired separation achieved (active distance {:.6e} >= {:.6e}). Stopping.",
                active_dist,
                sep);
            break;
        }

        if (active_dist > sep) {
            // Overshot: do NOT commit, so the next solve warm starts from the last accepted state.
            if (!line_search_step.has_value()) {
                log_and_throw_error("Cannot overshoot on first iteration if init_dhat=sep");
            }
            // `min(dhat_high or np.inf, ...)`: Python's `or` also falls through on a stored 0.0.
            const double high =
                (dhat_high.has_value() && *dhat_high != 0.0) ? *dhat_high : INF;
            dhat_high = std::min(high, curr_json["contact"]["dhat"].get<double>());
            alpha *= 0.5;
            curr_json["contact"]["dhat"] = committed_dhat + *line_search_step * alpha;
            logger().info(
                "Overshot target separation. Reverting to previous state. Reducing line search "
                "alpha to {:.6f}",
                alpha);
            logger().info(
                "Updated dhat to {:.6e} for next iteration",
                curr_json["contact"]["dhat"].get<double>());

            continue;
        } else {
            backend.commit_warm_start();
            // step is approved
            committed_dhat = curr_json["contact"]["dhat"].get<double>();
            alpha = 1.0; // reset line search
            if (prev_active.has_value() && active_dist < *prev_active * 1.01 && !stall_warned) {
                logger().info(
                    "Separation grew <1% this iteration — likely at the fixed-stiffness ceiling "
                    "(penalties balance the barrier); consider raising barrier_stiffness or "
                    "strategy=\"stiffness\".");
                stall_warned = true;
            }
            prev_active = active_dist;
            if (dhat_high.has_value() && *dhat_high - committed_dhat < 1e-3 * *dhat_high) {
                // bracket collapsed but still undershooting: the state has drifted since the
                // overshoot was recorded -- drop the bound
                dhat_high.reset();
            }
            double dhat_next = growth * active_dist;
            if (dhat_high.has_value() && dhat_next >= *dhat_high) {
                // a dhat >= dhat_high is already known to overshoot: bisect the bracket instead
                // of re-discovering it
                dhat_next = 0.5 * (committed_dhat + *dhat_high);
            }
            line_search_step = dhat_next - committed_dhat;
            curr_json["contact"]["dhat"] = committed_dhat + *line_search_step * alpha;
            logger().info(
                "Updated dhat to {:.6e} for next iteration",
                curr_json["contact"]["dhat"].get<double>());
        }
    }

    log_if_out_of_iterations(last_iter, n_iterations, active_dist);
    backend.reset_warm_start();
}

void run_polyfem_stiffness(
    PolyfemBackend& backend,
    OrderedJson& sep_json,
    const std::filesystem::path& sep_json_path,
    const std::filesystem::path& sim_out_dir,
    const OrderedJson& cfg)
{
    std::filesystem::create_directories(sim_out_dir);
    remove_stale_iteration_logs(sim_out_dir);

    double active_dist = -INF;
    const double sep = cfg.at("sep").get<double>();
    const double rtol = get_number(cfg, "rtol", opt_defaults().at("rtol").get<double>());
    const double dhat = sep * (1.0 + rtol);
    double kappa =
        get_number(cfg, "barrier_stiffness", opt_defaults().at("barrier_stiffness").get<double>());
    const double max_mult = get_number(cfg, "max_stiffness_multiplier", 100.0);

    sep_json["contact"]["dhat"] = dhat;

    OrderedJson& curr_json = sep_json;
    const WarmStartPaths state = open_warm_start(backend, curr_json, sim_out_dir);
    std::optional<double> prev_kappa;
    std::optional<double> prev_deficit;

    const int64_t n_iterations = max_iterations(cfg);
    require_one_iteration(n_iterations);
    // The Python's `for iter in range(...)` leaves its loop variable behind and reads it after
    // the loop; `last_iter` is that variable, and nothing else.
    int64_t last_iter = -1;
    for (int64_t iter = 0; iter < n_iterations; ++iter) {
        last_iter = iter;
        // This loop's own mutation, made before the document is written: `solve_iteration` writes
        // it, so the barrier stiffness has to be in it by then.
        curr_json["solver"]["contact"]["barrier_stiffness"] = kappa;
        const std::optional<double> parsed =
            solve_iteration(backend, curr_json, sep_json_path, sim_out_dir, state, iter);
        if (!parsed.has_value()) {
            break;
        }
        active_dist = *parsed;

        logger().info("Current active distance: {:.6e}  (kappa={:.6e})", active_dist, kappa);

        if (active_dist == INF) {
            logger().info("Active distance is infinite — no contact. Stopping.");
            break;
        }
        if (active_dist >= sep) {
            logger().info(
                "Desired separation achieved (active distance {:.6e} >= {:.6e}, bounded above by "
                "dhat={:.6e}). Stopping.",
                active_dist,
                sep,
                dhat);
            break;
        }

        // Always commit: the distance is monotone in kappa, no rollback.
        backend.commit_warm_start();

        const double deficit = dhat - active_dist; // > dhat - sep since active < sep
        const double target = 0.5 * (dhat - sep); // aim mid-tolerance for margin
        double exponent = -0.5; // theory: force ~ kappa*delta^2
        if (prev_deficit.has_value() && deficit < *prev_deficit && kappa > *prev_kappa) {
            const double measured =
                std::log(deficit / *prev_deficit) / std::log(kappa / *prev_kappa);
            if (measured < -1e-3) {
                exponent = measured;
            }
        }
        prev_kappa = kappa;
        prev_deficit = deficit;
        const double mult = std::min(std::pow(target / deficit, 1.0 / exponent), max_mult);
        kappa *= mult;
        logger().info(
            "Deficit {:.6e} (tolerance {:.6e}, exponent {:.3f}); raising barrier stiffness x{:.3g} "
            "to {:.6e}",
            deficit,
            dhat - sep,
            exponent,
            mult,
            kappa);
    }

    log_if_out_of_iterations(last_iter, n_iterations, active_dist);
    backend.reset_warm_start();
}

} // namespace wmtk::components::polyfem_ops
