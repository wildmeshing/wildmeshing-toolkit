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

TEST_CASE("symdir2d real cases")
{
    std::vector<Eigen::Vector3d> ref_coordinaites(3);
    ref_coordinaites[0] = {0.236773, 0.343805, -0.0856835};
    ref_coordinaites[1] = {0.238737, 0.348135, -0.0939082};
    ref_coordinaites[2] = {0.2407, 0.352464, -0.102133};

    Eigen::Vector2d a = {-0.372297, 1.51372};
    Eigen::Vector2d b = {-1.57569, 2.46899};
    Eigen::Vector2d c = {-2.77909, 3.42426};

    std::cout << wmtk::function::utils::symdir(
                     ref_coordinaites[0],
                     ref_coordinaites[1],
                     ref_coordinaites[2],
                     a,
                     b,
                     c)
              << std::endl;
    std::cout << wmtk::function::utils::symdir(
                     ref_coordinaites[1],
                     ref_coordinaites[2],
                     ref_coordinaites[0],
                     b,
                     c,
                     a)
              << std::endl;
    std::cout << wmtk::function::utils::symdir(
                     ref_coordinaites[2],
                     ref_coordinaites[0],
                     ref_coordinaites[1],
                     c,
                     a,
                     b)
              << std::endl;

    std::cout << wmtk::function::utils::symdir(
                     ref_coordinaites[2],
                     ref_coordinaites[1],
                     ref_coordinaites[0],
                     c,
                     b,
                     a)
              << std::endl;
    std::cout << wmtk::function::utils::symdir(
                     ref_coordinaites[1],
                     ref_coordinaites[0],
                     ref_coordinaites[2],
                     b,
                     a,
                     c)
              << std::endl;
    std::cout << wmtk::function::utils::symdir(
                     ref_coordinaites[0],
                     ref_coordinaites[2],
                     ref_coordinaites[1],
                     a,
                     c,
                     b)
              << std::endl;
}