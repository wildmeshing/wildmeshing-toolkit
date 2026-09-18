#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/polyfem_ops/NumpyCompat.hpp>
#include <wmtk/components/polyfem_ops/PolyfemRunner.hpp>

#include <limits>

using polysolve::nonlinear::Status;
using wmtk::components::polyfem_ops::check_polyfem_success;
using wmtk::components::polyfem_ops::numpy_isclose;

TEST_CASE("polyfem_ops check_polyfem_success statuses", "[components][polyfem_ops]")
{
    // The statuses polyfem recorded, one per subsolve. Any one accepted status is enough, as for
    // the Python any one accepted "Finished:" phrase anywhere in the executable's output is.
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
