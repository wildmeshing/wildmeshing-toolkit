#include <Eigen/Core>
#include <wmtk/utils/orient_triangle_2d_vertices.hpp>
#include <wmtk/utils/point_inside_triangle_check.hpp>
#include <wmtk/utils/triangle_areas.hpp>

#include <catch2/catch_test_macros.hpp>
#include <iostream>


using namespace wmtk::utils;
using namespace wmtk;

TEST_CASE("point_inside_tri")
{
    Eigen::Vector2d a(0, 0);
    Eigen::Vector2d b(1, 0);
    Eigen::Vector2d c(0, 1);
    Eigen::Vector2d p1(0.5, 0.5);
    REQUIRE(point_inside_triangle_2d_check(a, b, c, p1) == true);

    Eigen::Vector2d p2(0.2, 0.2);
    REQUIRE(point_inside_triangle_2d_check(a, b, c, p2) == true);

    Eigen::Vector2d p3(1, 1);
    REQUIRE(point_inside_triangle_2d_check(a, b, c, p3) == false);
}

TEST_CASE("orient_tri")
{
    Eigen::Vector2d a(0, 0);
    Eigen::Vector2d b(1, 0);
    Eigen::Vector2d c(0, 1);
    REQUIRE(orient2d(a.data(), b.data(), c.data()) > 0);
    Eigen::Vector2d a_oriented, b_oriented, c_oriented;
    orient_triangle_2d_vertices(a, b, c, a_oriented, b_oriented, c_oriented);

    REQUIRE(a_oriented == a);
    REQUIRE(b_oriented == b);
    REQUIRE(c_oriented == c);
    REQUIRE(orient2d(a_oriented.data(), b_oriented.data(), c_oriented.data()) > 0);

    Eigen::Vector2d a1(0, 0);
    Eigen::Vector2d b1(0, 1);
    Eigen::Vector2d c1(1, 0);
    REQUIRE(orient2d(a1.data(), b1.data(), c1.data()) < 0);
    orient_triangle_2d_vertices(a1, b1, c1, a_oriented, b_oriented, c_oriented);
    REQUIRE(a_oriented == a1);
    REQUIRE(b_oriented == c1);
    REQUIRE(c_oriented == b1);
    REQUIRE(orient2d(a_oriented.data(), b_oriented.data(), c_oriented.data()) > 0);
}