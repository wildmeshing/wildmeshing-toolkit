#include <wmtk/utils/DisplacementBicubic.h>
#include <wmtk/utils/DisplacementSpline.h>
#include <wmtk/utils/Energy2d.h>
#include <catch2/catch.hpp>
#include <iostream>

using namespace ::wmtk;
template <class T>
using RowMatrix2 = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Index = uint64_t;
using Scalar = double;

TEST_CASE("displacement_bicubic_constant", "[displacement]")
{
    Image image(8, 8);
    WrappingMode wrapping_mode = WrappingMode::MIRROR_REPEAT;

    SECTION("displacement == 0")
    {
        auto f_0 = []([[maybe_unused]] const double& u,
                      [[maybe_unused]] const double& v) -> double { return 0; };
        image.set(f_0, wrapping_mode, wrapping_mode);
    }
    SECTION("displacement == 42")
    {
        auto f_42 = []([[maybe_unused]] const double& u,
                       [[maybe_unused]] const double& v) -> double { return 42; };
        image.set(f_42, wrapping_mode, wrapping_mode);
    }
    DisplacementBicubic displ(image);

    // Make sure that image.get(), displ.get(), and f() all give the same result
    for (float u = 0; u <= 1; u += 0.1) {
        for (float v = 0; v <= 1; v += 0.1) {
            REQUIRE_THAT(image.get(u, v), Catch::Matchers::WithinRel(displ.get(u, v)));
        }
    }
}

TEST_CASE("displacement_bicubic_linear", "[displacement]")
{
    WrappingMode wrapping_mode = WrappingMode::MIRROR_REPEAT;
    Image image(8, 8);

    auto f_linear = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    image.set(f_linear, wrapping_mode, wrapping_mode);

    DisplacementBicubic displ(image);

    // Make sure that image.get(), displ.get(), and f() all give the same result
    for (float u = 0; u <= 1; u += 0.1) {
        for (float v = 0; v <= 1; v += 0.1) {
            REQUIRE(image.get(u, v) == displ.get(u, v));
        }
    }

    std::cout << "Img = " << image.get(0.5, 0.5) << std::endl;
    std::cout << "f() = " << f_linear(0.5, 0.5) << std::endl;

    REQUIRE_THAT(image.get(0.5, 0.5), Catch::Matchers::WithinRel(f_linear(0.5, 0.5), 1e-5));
}

TEST_CASE("displacement_spline_constant", "[displacement]")
{
    Image image(8, 8);
    WrappingMode wrapping_mode = WrappingMode::MIRROR_REPEAT;

    SECTION("displacement == 0")
    {
        auto f_0 = []([[maybe_unused]] const double& u,
                      [[maybe_unused]] const double& v) -> double { return 0; };
        image.set(f_0, wrapping_mode, wrapping_mode);
    }
    SECTION("displacement == 42")
    {
        auto f_42 = []([[maybe_unused]] const double& u,
                       [[maybe_unused]] const double& v) -> double { return 42; };
        image.set(f_42, wrapping_mode, wrapping_mode);
    }
    DisplacementSpline displ(image);

    // Make sure that image.get(), displ.get(), and f() all give the same result
    for (double u = 0; u <= 1; u += 0.1) {
        for (double v = 0; v <= 1; v += 0.1) {
            std::cout << "Img = " << image.get(u, v) << ", Spline = " << displ.get(u, v)
                      << std::endl;
            REQUIRE_THAT(image.get(u, v), Catch::Matchers::WithinRel(displ.get(u, v), 1e-5));
        }
    }
}

TEST_CASE("displacement_spline_linear", "[displacement]")
{
    WrappingMode wrapping_mode = WrappingMode::MIRROR_REPEAT;
    Image image(20, 20);

    auto f_linear = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    image.set(f_linear, wrapping_mode, wrapping_mode);

    DisplacementSpline displ(image);
    // displ.set_wrapping_mode(wrapping_mode, wrapping_mode);

    // Make sure that displ.get(), and f() give the same result on the interior
    for (double u = 0.2; u <= 0.8; u += 0.1) {
        for (double v = 0.2; v <= 0.8; v += 0.1) {
            std::cout << "u, v = " << u << ", " << v << ", f() = " << f_linear(u, v)
                      << ", Spline = " << displ.get(u, v) << std::endl;
            REQUIRE_THAT(image.get(u, v), Catch::Matchers::WithinRel(displ.get(u, v), 1e-5));
        }
    }
}

TEST_CASE("bicubic interpolation")
{
    // Create a random bivariate cubic polynomial
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist_coeffs(-0.5f, 0.5f);

    Eigen::Matrix4f M = Eigen::Matrix4f::NullaryExpr([&]() { return dist_coeffs(gen); });
    auto bivariate_cubic_polynomial = [&](double x, double y) -> double {
        Eigen::Vector4f X(1, x, x * x, x * x * x);
        Eigen::Vector4f Y(1, y, y * y, y * y * y);
        return X.transpose() * M * Y;
    };

    // Create an image from the polynomial function
    int width = 40;
    int height = 40;
    Image image(width, height);
    image.set(bivariate_cubic_polynomial);

    // Sample random points in the image and check correctness.
    // Avoid image boundaries since finite diff will be off at image border.
    std::uniform_real_distribution<float> dist_samples(0.1, 0.9);

    for (size_t k = 0; k < 100; ++k) {
        Eigen::Vector2d p(dist_samples(gen), dist_samples(gen));
        auto value0 = image.get(p.x(), p.y());
        auto expected = bivariate_cubic_polynomial(p.x(), p.y());
        CAPTURE(k, p.x(), p.y(), value0, expected);
        REQUIRE_THAT(value0, Catch::Matchers::WithinRel(expected, 0.001));
    }
}

TEST_CASE("bicubic autodiff")
{
    // Create a random bivariate cubic polynomial
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist_coeffs(-0.5f, 0.5f);

    Eigen::Matrix4f M = Eigen::Matrix4f::NullaryExpr([&]() { return dist_coeffs(gen); });
    auto bivariate_cubic_polynomial = [&](auto x, auto y) {
        using T = std::decay_t<decltype(x)>;
        Eigen::Vector4<T> X(T(1), x, x * x, x * x * x);
        Eigen::Vector4<T> Y(T(1), y, y * y, y * y * y);
        return T(X.transpose() * M.cast<T>() * Y);
    };

    // Create an image from the polynomial function
    int width = 40;
    int height = 40;
    Image image(width, height);
    image.set(bivariate_cubic_polynomial);

    // Sample random points in the image and check correctness.
    std::uniform_real_distribution<float> dist_samples(0.1, 0.9);

    DiffScalarBase::setVariableCount(2);
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
    for (size_t k = 0; k < 100; ++k) {
        Eigen::Vector2d p(dist_samples(gen), dist_samples(gen));
        DScalar x(0, p.x());
        DScalar y(1, p.y());
        DScalar value0 = image.get(x, y);
        DScalar expected = bivariate_cubic_polynomial(x, y);
        auto g0 = value0.getGradient().eval();
        auto g1 = expected.getGradient().eval();
        auto h0 = value0.getHessian().eval();
        auto h1 = expected.getHessian().eval();

        auto i = static_cast<int>(std::floor(p.x() * width - Scalar(0.5)));
        auto j = static_cast<int>(std::floor(p.y() * height - Scalar(0.5)));
        CAPTURE(k, i, j, p.x(), p.y(), value0, expected);
        REQUIRE_THAT(value0.getValue(), Catch::Matchers::WithinRel(expected.getValue(), 1e-2));
        for (int w = 0; w < g0.size(); ++w) {
            REQUIRE_THAT(g0[w], Catch::Matchers::WithinRel(g1[w], 1e-3));
        }
        for (int w = 0; w < h0.size(); ++w) {
            REQUIRE_THAT(
                h0.data()[w],
                Catch::Matchers::WithinRel(h1.data()[w], 1e-2) ||
                    Catch::Matchers::WithinAbs(0, 1e-3));
        }
    }
}

TEST_CASE("bicubic periodic")
{
    // Use periodic analytical function
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist_coeffs(-0.5f, 0.5f);

    Eigen::Vector2f coeffs = Eigen::Vector2f::NullaryExpr([&]() { return dist_coeffs(gen); });
    auto periodic_function = [&](auto x, auto y) {
        using T = std::decay_t<decltype(x)>;
        T z = coeffs[0] * sin(x * 2.f * M_PI) + coeffs[1] * sin(y * 2.f * M_PI);
        return z;
    };

    // Create an image from the polynomial function
    int width = 40;
    int height = 40;
    Image image(width, height);
    image.set(periodic_function, WrappingMode::REPEAT, WrappingMode::REPEAT);

    // Sample random points in the image and check correctness.
    std::uniform_real_distribution<float> dist_samples(0, 1);

    DiffScalarBase::setVariableCount(2);
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
    for (size_t k = 0; k < 100; ++k) {
        Eigen::Vector2d p(dist_samples(gen), dist_samples(gen));
        DScalar x(0, p.x());
        DScalar y(1, p.y());
        DScalar value0 = image.get(x, y);
        DScalar expected = periodic_function(x, y);
        auto g0 = value0.getGradient().eval();
        auto g1 = expected.getGradient().eval();
        auto h0 = value0.getHessian().eval();
        auto h1 = expected.getHessian().eval();

        auto i = static_cast<int>(std::floor(p.x() * width - Scalar(0.5)));
        auto j = static_cast<int>(std::floor(p.y() * height - Scalar(0.5)));
        CAPTURE(k, i, j, p.x(), p.y(), value0, expected);

        // Since the function is not truly a cubic polynomial, we relax our gradient/hessian
        // tolerance to a much lower value... but hopefully this is enough to check that our signal
        // is periodic over the image.
        REQUIRE_THAT(value0.getValue(), Catch::Matchers::WithinRel(expected.getValue(), 1e-2));
        for (int w = 0; w < g0.size(); ++w) {
            REQUIRE_THAT(g0[w], Catch::Matchers::WithinRel(g1[w], 1e-1));
        }
        for (int w = 0; w < h0.size(); ++w) {
            REQUIRE_THAT(
                h0.data()[w],
                Catch::Matchers::WithinRel(h1.data()[w], 1e-1) ||
                    Catch::Matchers::WithinAbs(0, 1e-3));
        }
    }
}

TEST_CASE("displacement_bicubic_quadrature")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;
    DiffScalarBase::setVariableCount(2);
    Image image(100, 100);
    auto displacement_double = []([[maybe_unused]] const double& u,
                                  [[maybe_unused]] const double& v) -> double { return 10; };
    image.set(displacement_double, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    std::mt19937 rand_generator;
    std::uniform_real_distribution<double> rand_dist(0.1, 0.9); // not too close to the boundary
    DisplacementBicubic displ = DisplacementBicubic(image);
    for (int i = 0; i < 10; i++) {
        Eigen::Vector2d p1, p2;
        p1 = Eigen::Vector2d(rand_dist(rand_generator), rand_dist(rand_generator));
        p2 = Eigen::Vector2d(rand_dist(rand_generator), rand_dist(rand_generator));
        REQUIRE(displ.get_error_per_edge(p1, p2) < 1e-7);
    }
    wmtk::logger().info("============= 10x =============");

    auto displacement_double2 = [](const double& u, [[maybe_unused]] const double& v) -> float {
        return static_cast<float>(10 * u); // sin(M_PI * u);
    };
    image.set(displacement_double2, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    displ.set_image(image);
    for (int i = 0; i < 10; i++) {
        Eigen::Vector2d p1, p2;
        p1 = Eigen::Vector2d(rand_dist(rand_generator), rand_dist(rand_generator));
        p2 = Eigen::Vector2d(rand_dist(rand_generator), rand_dist(rand_generator));
        auto edge_error = displ.get_error_per_edge(p1, p2);
        REQUIRE(edge_error < 1e-6);
    }

    wmtk::logger().info("============= sin(PI * u) =============");

    auto displacement_double3 = [](const double& u, [[maybe_unused]] const double& v) -> float {
        return static_cast<float>(sin(M_PI * u));
    };
    image.set(displacement_double3, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    displ.set_image(image);
    Eigen::Vector2d v1, v2;
    v1 << 0.1, 0.;
    v2 << 0.9, 0.;
    auto edge_error = displ.get_error_per_edge(v1, v2);
    wmtk::logger().info("final edge error {} ", edge_error);
    REQUIRE(abs(edge_error - 0.358247787412568) < 1e-8);
    v1 = Eigen::Vector2d(0, 0.1);
    v2 = Eigen::Vector2d(0, 0.9);
    edge_error = displ.get_error_per_edge(v1, v2);
    wmtk::logger().info("final edge error {} ", edge_error);
    REQUIRE(abs(edge_error) < 1e-8);
}
