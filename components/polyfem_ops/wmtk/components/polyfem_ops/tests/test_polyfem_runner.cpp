#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/polyfem_ops/NumpyCompat.hpp>
#include <wmtk/components/polyfem_ops/PolyfemRunner.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using polysolve::nonlinear::Status;
using wmtk::components::polyfem_ops::check_polyfem_success;
using wmtk::components::polyfem_ops::numpy_isclose;
using wmtk::components::polyfem_ops::parse_active_distance;
using wmtk::components::polyfem_ops::statuses_from_log;

namespace {

/// A captured solve, as `run_streaming` returns it: one string per line, newline included.
std::vector<std::string> lines(const std::vector<std::string>& raw)
{
    std::vector<std::string> out;
    for (const auto& line : raw) {
        out.push_back(line + "\n");
    }
    return out;
}

/// Real lines from `sep_output/polyfem_iter_0.log` of a boxes3d separation run, trimmed to the
/// ones the two readers look at. The value is polyfem's own 17-significant-digit print.
const std::vector<std::string>& real_log()
{
    static const std::vector<std::string> captured = lines(
        {"[2026-09-17 21:41:45.625] [polyfem] [info] Solver Eigen::AccelerateLDLT is the highest "
         "priority available solver; using it.",
         "[2026-09-17 21:41:46.998] [polyfem] [debug] Minimum distance during solve: "
         "0.001744855718822498, active distance: 0.001744855718822498, dhat: 0.0019",
         "[2026-09-17 21:41:47.002] [polyfem] [debug] Minimum distance during solve: "
         "0.0017448556942542116, active distance: 0.0017448556942542119, dhat: 0.0019",
         "[2026-09-17 21:41:47.003] [polyfem] [info] [SparseNewton][RobustArmijo] Finished: "
         "Relative gradient vector too small took 0.0773682s (iters=15)"});
    return captured;
}

/// The subprocess backend's path: the statuses are recovered from the captured lines.
void check_log(int returncode, const std::vector<std::string>& log, bool allow_out_of_iterations)
{
    check_polyfem_success(returncode, statuses_from_log(log), log, allow_out_of_iterations);
}

} // namespace

TEST_CASE("polyfem_ops check_polyfem_success phrases", "[components][polyfem_ops]")
{
    // The two phrases the Python accepts unconditionally.
    CHECK_NOTHROW(check_log(0, real_log(), false));
    CHECK_NOTHROW(check_log(
        0,
        lines({"[polyfem] [info] Finished: Gradient vector norm too small took 1s"}),
        false));

    // The iteration limit counts only with the allowance, which is what every solve this port
    // launches passes (the built JSON always sets solver.nonlinear.allow_out_of_iterations).
    const auto out_of_iterations =
        lines({"[polyfem] [info] Finished: Iteration limit reached took 12s (iters=1000)"});
    CHECK_THROWS(check_log(0, out_of_iterations, false));
    CHECK_NOTHROW(check_log(0, out_of_iterations, true));

    // A real failure: an accepted phrase but a non-zero exit code, and a rejected phrase.
    CHECK_THROWS(check_log(1, real_log(), true));
    CHECK_THROWS(check_log(
        0,
        lines({"[polyfem] [warning] Finished: Not descent direction", "[polyfem] [error] failed"}),
        true));
    // No "Finished:" line at all -- the Python reports the missing line as None and still throws.
    CHECK_THROWS(check_log(0, lines({"[polyfem] [info] reading mesh"}), true));
}

TEST_CASE("polyfem_ops check_polyfem_success statuses", "[components][polyfem_ops]")
{
    // The in-process backend's path: the statuses polyfem recorded, one per subsolve. Any one
    // accepted status is enough, as any one accepted phrase is in the log.
    CHECK_NOTHROW(check_polyfem_success(0, {Status::RelGradNormTolerance}, {}, false));
    CHECK_NOTHROW(check_polyfem_success(
        0,
        {Status::LineSearchFailed, Status::GradNormTolerance},
        {},
        false));
    CHECK_THROWS(check_polyfem_success(0, {Status::IterationLimit}, {}, false));
    CHECK_NOTHROW(check_polyfem_success(0, {Status::IterationLimit}, {}, true));
    CHECK_THROWS(check_polyfem_success(1, {Status::RelGradNormTolerance}, {}, true));
    CHECK_THROWS(check_polyfem_success(0, {Status::FDeltaTolerance}, {}, true));
    CHECK_THROWS(check_polyfem_success(0, {}, {}, true));
}

TEST_CASE("polyfem_ops parse_active_distance", "[components][polyfem_ops]")
{
    // The LAST matching line wins: polyfem logs one after every Newton step and the converged
    // state is the last of them.
    const auto parsed = parse_active_distance(real_log());
    REQUIRE(parsed.has_value());
    CHECK(*parsed == 0.0017448556942542119);

    // The trailing comma is stripped, not parsed, and no digit is lost: the token is converted as
    // it stands, so the value is bit-for-bit the one polyfem printed.
    CHECK(*parse_active_distance(lines({"active distance: 0.0017448556942542119, dhat: 0.0019"})) ==
          0.0017448556942542119);
    CHECK(*parse_active_distance(lines({"active distance: 1.5e-3;"})) == 1.5e-3);

    // polyfem reports an out-of-range gap as inf, which both loops test for explicitly.
    CHECK(std::isinf(*parse_active_distance(lines({"active distance: inf, dhat: 0.0019"}))));

    // Nothing to read: the loops stop with "contact not triggered" instead of guessing.
    CHECK_FALSE(parse_active_distance(lines({"[polyfem] [info] reading mesh"})).has_value());
    CHECK_FALSE(parse_active_distance({}).has_value());
}

TEST_CASE("polyfem_ops numpy_isclose mirror", "[components][polyfem_ops]")
{
    // |a - b| <= atol + rtol * |b|, with numpy's default atol of 1e-8.
    const double sep = 1.5e-3;
    const double rtol = 1e-1;
    CHECK(numpy_isclose(sep * (1.0 + rtol), sep, rtol)); // exactly on the band
    CHECK(numpy_isclose(sep * (1.0 + 2.0 * rtol), sep, rtol) == false);

    // The atol floor is not negligible at these magnitudes: at rtol 1e-6 the band around
    // sep = 1.5e-3 is 1e-8 + 1.5e-9, so a 1e-8 difference is still "close".
    CHECK(numpy_isclose(sep + 1e-8, sep, 1e-6));
    CHECK_FALSE(numpy_isclose(sep + 1e-7, sep, 1e-6));

    // Asymmetric, because the tolerance is built from the SECOND argument alone: the same gap of
    // 10 is within 10% of 100 but not within 10% of 90. Checked against numpy 2.4.6.
    CHECK(numpy_isclose(90.0, 100.0, 0.1));
    CHECK_FALSE(numpy_isclose(100.0, 90.0, 0.1));

    // NaN is never close, as in numpy with equal_nan=False.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(numpy_isclose(nan, sep, rtol));
    CHECK_FALSE(numpy_isclose(sep, nan, rtol));
}
