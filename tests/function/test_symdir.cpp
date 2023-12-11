#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <wmtk/function/utils/symdir.hpp>

TEST_CASE("symdir2d")
{
    Eigen::Vector3d V0 = {0, 0, 0};
    Eigen::Vector3d V1 = {1, 0, 0};
    Eigen::Vector3d V2 = {0.5, std::sqrt(3) / 2, 0};
    SECTION("equilateral_triangle")
    {
        Eigen::Vector2d uv0 = {0, 0};
        Eigen::Vector2d uv1 = {1, 0};
        Eigen::Vector2d uv2 = {0.5, std::sqrt(3) / 2};

        CHECK(wmtk::function::utils::symdir(V0, V1, V2, uv0, uv1, uv2) == 4.0);
    }
    SECTION("equilateral_triangle x 2")
    {
        Eigen::Vector2d uv0 = {0, 0};
        Eigen::Vector2d uv1 = {2, 0};
        Eigen::Vector2d uv2 = {1, std::sqrt(3)};

        CHECK(wmtk::function::utils::symdir(V0, V1, V2, uv0, uv1, uv2) == 8.5);
    }
    SECTION("equilateral_triangle/3")
    {
        Eigen::Vector2d uv0 = {0, 0};
        Eigen::Vector2d uv1 = {1 / 3.0, 0};
        Eigen::Vector2d uv2 = {1 / 6.0, std::sqrt(3) / 6.0};

        CHECK(wmtk::function::utils::symdir(V0, V1, V2, uv0, uv1, uv2) - 18 - 2.0 / 9.0 < 1e-10);
    }
}