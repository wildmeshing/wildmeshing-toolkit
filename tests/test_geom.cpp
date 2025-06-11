#include <wmtk/utils/GeoUtils.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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

    bool inter = open_segment_triangle_intersection_3d(seg, tri, p);
    REQUIRE(inter);

    REQUIRE(p[0] == Catch::Approx(0));
    REQUIRE(p[1] == Catch::Approx(0));
    REQUIRE(p[2] == Catch::Approx(1));
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

    bool inter;

    // bool inter = is_point_inside_triangle(Vector2f(0.5, 0.5), tri);
    // REQUIRE(!inter);

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


TEST_CASE("open_segment_open_segment_intersection_2d", "[test_geom]")
{
    const auto permute = [](const std::array<Vector2d, 2>& seg1,
                            const std::array<Vector2d, 2>& seg2,
                            bool val,
                            std::array<double, 2> expected_t) {
        const std::array<std::array<Vector2d, 2>, 2> segs = {{seg1, seg2}};

        for (int curr_seg = 0; curr_seg <= 1; ++curr_seg) {
            for (int d = 0; d <= 1; ++d) {
                const std::array<Vector2d, 2> s1 = {
                    {segs[curr_seg][d], segs[curr_seg][(d + 1) % 2]}};
                const std::array<Vector2d, 2> s2 = {
                    {segs[(curr_seg + 1) % 2][d], segs[(curr_seg + 1) % 2][(d + 1) % 2]}};

                double t;
                const auto res = open_segment_open_segment_intersection_2d(s1, s2, t);
                REQUIRE(res == val);
                if (val) {
                    if (d == 0)
                        REQUIRE(t == Catch::Approx(expected_t[curr_seg]));
                    else
                        REQUIRE(t == Catch::Approx(1 - expected_t[curr_seg]));
                } else
                    REQUIRE(t == -1);
            }
        }
    };

    SECTION("Simple case, inter")
    {
        const std::array<Vector2d, 2> seg1 = {{
            Vector2d(0, 0),
            Vector2d(0, 3),
        }};

        const std::array<Eigen::Vector2d, 2> seg2 = {{
            Vector2d(-2, 1),
            Vector2d(1, 1),
        }};

        permute(seg1, seg2, true, {{1. / 3., 2. / 3.}});
    }

    SECTION("Simple case, no inter")
    {
        const std::array<Vector2d, 2> seg1 = {{
            Vector2d(0, 0),
            Vector2d(0, 2),
        }};

        const std::array<Eigen::Vector2d, 2> seg2 = {{
            Vector2d(-2, 1),
            Vector2d(-1, 1),
        }};

        permute(seg1, seg2, false, {{-1, -1}});
    }

    SECTION("Point on, no inter")
    {
        const std::array<Vector2d, 2> seg1 = {{
            Vector2d(0, 0),
            Vector2d(0, 2),
        }};

        const std::array<Eigen::Vector2d, 2> seg2 = {{
            Vector2d(-2, 1),
            Vector2d(0, 1),
        }};

        permute(seg1, seg2, false, {{-1, -1}});
    }


    SECTION("collinar no inter")
    {
        const std::array<Vector2d, 2> seg1 = {{
            Vector2d(0, 0),
            Vector2d(5, 5),
        }};

        const std::array<Eigen::Vector2d, 2> seg2 = {{
            Vector2d(-2, -2),
            Vector2d(-5, -5),
        }};

        permute(seg1, seg2, false, {{-1, -1}});
    }

    SECTION("collinar inter")
    {
        const std::array<Vector2d, 2> seg1 = {{
            Vector2d(0, 0),
            Vector2d(5, 5),
        }};

        const std::array<Eigen::Vector2d, 2> seg2 = {{
            Vector2d(2, 2),
            Vector2d(-5, -5),
        }};

        permute(seg1, seg2, false, {{-1, -1}});
    }
}


TEST_CASE("segment_triangle_coplanar_3d", "[test_geom]")
{
    const std::array<Vector3d, 2> seg = {{
        Vector3d(0, 0, 1),
        Vector3d(0, 3, 1),
    }};

    const std::array<Eigen::Vector3d, 3> tri = {{
        Vector3d(-1, -1, 1),
        Vector3d(1, 0, 1),
        Vector3d(0, 1, 1),
    }};

    const auto res = segment_triangle_coplanar_3d(seg, tri);
    REQUIRE(res);
}