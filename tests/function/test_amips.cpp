#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <wmtk/function/utils/amips.hpp>

TEST_CASE("amips2d", "[function]")
{
    SECTION("equilateral_triangle")
    {
        Eigen::Vector2d uv0 = {0, 0};
        Eigen::Vector2d uv1 = {1, 0};
        Eigen::Vector2d uv2 = {0.5, std::sqrt(3) / 2};

        CHECK(wmtk::function::utils::amips(uv0, uv1, uv2) == 2.0);
    }
}

TEST_CASE("amips3d", "[function][.]")
{
    // TODOfix: this is not testing for 3D AMIPS...
    SECTION("equilateral_triangle")
    {
        Eigen::Vector2d uv0 = {0, 0};
        Eigen::Vector2d uv1 = {1, 0};
        Eigen::Vector2d uv2 = {0.5, std::sqrt(3) / 2};

        CHECK(wmtk::function::utils::amips(uv0, uv1, uv2) == 2.0);
    }
}
