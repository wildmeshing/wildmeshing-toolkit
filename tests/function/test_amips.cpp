#include <iostream>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/function/utils/amips.hpp>

TEST_CASE("amips2d")
{
    SECTION("equilateral_triangle")
    {
        Eigen::Vector2d uv0 = {0,0};
        Eigen::Vector2d uv1 = {1,0};
        Eigen::Vector2d uv2 = {0.5,std::sqrt(3)/2};


        std::cout << uv0 << std::endl;

        CHECK(wmtk::function::utils::amips(uv0,uv1,uv2) == 2.0);
    }
}

TEST_CASE("amips3d")
{
    SECTION("equilateral_triangle")
    {
        Eigen::Vector2d uv0 = {0,0};
        Eigen::Vector2d uv1 = {1,0};
        Eigen::Vector2d uv2 = {0.5,std::sqrt(3)/2};

        CHECK(wmtk::function::utils::amips(uv0,uv1,uv2) == 2.0);
    }
}
