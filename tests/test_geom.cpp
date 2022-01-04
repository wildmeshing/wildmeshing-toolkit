#include <wmtk/utils/GeoUtils.h>

#include <catch2/catch.hpp>

using namespace wmtk;
using namespace Eigen;

TEST_CASE("test_segment_triangle_intersection", "[test_geom]")
{
    const std::array<Vector3d, 2> seg = {{
        Vector3d(0, 0, 0),
        Vector3d(0, 0, 2),
    }};

    const std::array<Eigen::Vector3d, 3> tri = {{
        Vector3d(-1, -1, 1),
        Vector3d(1, 0, 1),
        Vector3d(0, 1, 1),
    }};
    Eigen::Vector3d p;

    bool inter = segment_triangle_intersection(seg, tri, p);
    REQUIRE(inter);

    REQUIRE(p[0] == Approx(0));
    REQUIRE(p[1] == Approx(0));
    REQUIRE(p[2] == Approx(1));
}

TEST_CASE("is_point_inside_triangle", "[test_geom]")
{
    const std::array<Eigen::Vector2d, 3> tri = {{
        Vector2d(-1, 0),
        Vector2d(1, 0),
        Vector2d(0, 1),
    }};

    bool inter = is_point_inside_triangle(Vector2d(0.5, 0.5), tri);
    REQUIRE(inter);

    inter = is_point_inside_triangle(Vector2d(5, 5), tri);
    REQUIRE(!inter);

    inter = is_point_inside_triangle(Vector2d(-1, 0), tri);
    REQUIRE(inter);

    inter = is_point_inside_triangle(Vector2d(0.5, 0), tri);
    REQUIRE(inter);

    inter = is_point_inside_triangle(Vector2d(0, 0.2), tri);
    REQUIRE(inter);

    inter = is_point_inside_triangle(Vector2d(0.2, 1e-16), tri);
    REQUIRE(inter);

    inter = is_point_inside_triangle(Vector2d(0.2, -1e-16), tri);
    REQUIRE(!inter);
}

TEST_CASE("is_point_inside_trianglef", "[test_geom]")
{
    const std::array<Eigen::Vector2f, 3> tri = {{
        Vector2f(-1, 0),
        Vector2f(1, 0),
        Vector2f(0, 1),
    }};

    bool inter = is_point_inside_triangle(Vector2f(0.5, 0.5), tri);
    REQUIRE(inter);

    inter = is_point_inside_triangle(Vector2f(5, 5), tri);
    REQUIRE(!inter);

    // these might fail due to numerics, double uses predicates!
    //  inter = is_point_inside_triangle(Vector2f(-1, 0), tri);
    //  REQUIRE(inter);

    // inter = is_point_inside_triangle(Vector2f(0.5, 0), tri);
    // REQUIRE(inter);

    // inter = is_point_inside_triangle(Vector2f(0, 0.2), tri);
    // REQUIRE(inter);

    inter = is_point_inside_triangle(Vector2f(0.2, 1e-6), tri);
    REQUIRE(inter);

    inter = is_point_inside_triangle(Vector2f(0.2, -1e-6), tri);
    REQUIRE(!inter);
}