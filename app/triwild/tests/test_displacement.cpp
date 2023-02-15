#include <wmtk/utils/DisplacementBicubic.h>
#include <wmtk/utils/DisplacementSpline.h>
#include <catch2/catch.hpp>
#include <iostream>

using namespace ::wmtk;

TEST_CASE("displacement_bicubic_constant", "[displacement]")
{
    Image image(8, 8);
    WrappingMode wrapping_mode = WrappingMode::MIRROR_REPEAT;

    SECTION("displacement == 0")
    {
        auto f_0 = [](const double& u, const double& v) -> double { return 0; };
        image.set(f_0, wrapping_mode, wrapping_mode);
    }
    SECTION("displacement == 42")
    {
        auto f_42 = [](const double& u, const double& v) -> double { return 42; };
        image.set(f_42, wrapping_mode, wrapping_mode);
    }
    DisplacementBicubic displ(image, wrapping_mode, wrapping_mode);

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

    auto f_linear = [](const double& u, const double& v) -> double { return u; };
    image.set(f_linear, wrapping_mode, wrapping_mode);

    DisplacementBicubic displ(image, wrapping_mode, wrapping_mode);

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
        auto f_0 = [](const double& u, const double& v) -> double { return 0; };
        image.set(f_0, wrapping_mode, wrapping_mode);
    }
    SECTION("displacement == 42")
    {
        auto f_42 = [](const double& u, const double& v) -> double { return 42; };
        image.set(f_42, wrapping_mode, wrapping_mode);
    }
    DisplacementSpline displ(image, wrapping_mode, wrapping_mode);

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
    Image image(20,20);

    auto f_linear = [](const double& u, const double& v) -> double { return u; };
    image.set(f_linear, wrapping_mode, wrapping_mode);

    DisplacementSpline displ(image, wrapping_mode, wrapping_mode);
    //displ.set_wrapping_mode(wrapping_mode, wrapping_mode);

    // Make sure that displ.get(), and f() give the same result on the interior
    for (double u = 0.2; u <= 0.8; u += 0.1) {
        for (double v = 0.2; v <= 0.8; v += 0.1) {
            std::cout << "u, v = " << u << ", " << v << ", f() = " << f_linear(u, v) << ", Spline = " << displ.get(u, v)
                      << std::endl;
            REQUIRE_THAT(image.get(u, v), Catch::Matchers::WithinRel(displ.get(u, v), 1e-5));
        }
    }
}