#include <wmtk/utils/Delaunay.hpp>

#include <catch2/catch_test_macros.hpp>

#include <limits>

namespace {
// TODO: this should use a predicate instead
inline double compute_tet_volume(
    const wmtk::delaunay::Point3D& p,
    const wmtk::delaunay::Point3D& q,
    const wmtk::delaunay::Point3D& r,
    const wmtk::delaunay::Point3D& s)
{
    return -(
        p[0] * q[1] * r[2] - p[0] * q[1] * s[2] - p[0] * q[2] * r[1] + p[0] * q[2] * s[1] +
        p[0] * r[1] * s[2] - p[0] * r[2] * s[1] - p[1] * q[0] * r[2] + p[1] * q[0] * s[2] +
        p[1] * q[2] * r[0] - p[1] * q[2] * s[0] - p[1] * r[0] * s[2] + p[1] * r[2] * s[0] +
        p[2] * q[0] * r[1] - p[2] * q[0] * s[1] - p[2] * q[1] * r[0] + p[2] * q[1] * s[0] +
        p[2] * r[0] * s[1] - p[2] * r[1] * s[0] - q[0] * r[1] * s[2] + q[0] * r[2] * s[1] +
        q[1] * r[0] * s[2] - q[1] * r[2] * s[0] - q[2] * r[0] * s[1] + q[2] * r[1] * s[0]);
}
// TODO: this should use a predicate instead
inline double compute_tri_area(
    const wmtk::delaunay::Point2D& a,
    const wmtk::delaunay::Point2D& b,
    const wmtk::delaunay::Point2D& c)
{
    auto a0 = a[0];
    auto a1 = a[1];
    auto b0 = b[0];
    auto b1 = b[1];
    auto c0 = c[0];
    auto c1 = c[1];

    return (-(a1 * b0) + a0 * b1 + a1 * c0 - b1 * c0 - a0 * c1 + b0 * c1) / 2.0;
}

} // namespace


TEST_CASE("Delaunay3D", "[delaunay][3d]")
{
    using namespace wmtk;

    auto validate = [](const auto& vertices, const auto& tets) {
        constexpr double EPS = std::numeric_limits<double>::epsilon();
        const size_t num_vertices = vertices.size();
        const size_t num_tets = tets.size();

        for (size_t i = 0; i < num_tets; i++) {
            const auto& tet = tets[i];
            REQUIRE(tet[0] < num_vertices);
            REQUIRE(tet[1] < num_vertices);
            REQUIRE(tet[2] < num_vertices);
            REQUIRE(tet[3] < num_vertices);

            // Tet must be positive oriented and non-degenerate.
            REQUIRE(
                compute_tet_volume(
                    vertices[tet[0]],
                    vertices[tet[1]],
                    vertices[tet[2]],
                    vertices[tet[3]]) > EPS);
        }
    };

    SECTION("Simple")
    {
        std::vector<delaunay::Point3D> points{{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}};

        auto [vertices, tets] = delaunay::delaunay3D(points);
        REQUIRE(vertices.size() == 4);
        REQUIRE(tets.size() == 1);
        validate(vertices, tets);
    }
    SECTION("Insufficient points should not fail")
    {
        std::vector<delaunay::Point3D> points{{{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}};

        auto [vertices, tets] = delaunay::delaunay3D(points);
        REQUIRE(tets.size() == 0);
        validate(vertices, tets);
    }
    SECTION("Coplanar pts")
    {
        std::vector<delaunay::Point3D> points{
            {{0, 0, 0}},
            {{1, 0, 0}},
            {{0, 1, 0}},
            {{0.5, 0.5, 0}}};

        auto [vertices, tets] = delaunay::delaunay3D(points);
        REQUIRE(vertices.size() == 4);
        REQUIRE(tets.size() == 0);
        validate(vertices, tets);
    }
    SECTION("Duplicate pts")
    {
        std::vector<delaunay::Point3D>
            points{{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}, {{0, 1, 0}}, {{0, 1, 0}}};

        auto [vertices, tets] = delaunay::delaunay3D(points);
        REQUIRE(vertices.size() == 6); // duplicate pts are kept in the output.
        REQUIRE(tets.size() == 1);
        validate(vertices, tets);
    }
    SECTION("Triangle prism")
    {
        std::vector<delaunay::Point3D>
            points{{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}, {{1, 0, 1}}, {{0, 1, 1}}};

        auto [vertices, tets] = delaunay::delaunay3D(points);
        REQUIRE(vertices.size() == 6);
        REQUIRE(tets.size() == 3);
        validate(vertices, tets);
    }
    SECTION("Cube with centroid")
    {
        std::vector<delaunay::Point3D> points{
            {{0, 0, 0}},
            {{1, 0, 0}},
            {{1, 1, 0}},
            {{0, 1, 0}},
            {{0, 0, 1}},
            {{1, 0, 1}},
            {{1, 1, 1}},
            {{0, 1, 1}},
            {{0.5, 0.5, 0.5}}};

        auto [vertices, tets] = delaunay::delaunay3D(points);
        REQUIRE(vertices.size() == 9);
        REQUIRE(tets.size() == 12);
        validate(vertices, tets);
    }
    SECTION("Cube with face center")
    {
        std::vector<delaunay::Point3D> points{
            {{0, 0, 0}},
            {{1, 0, 0}},
            {{1, 1, 0}},
            {{0, 1, 0}},
            {{0, 0, 1}},
            {{1, 0, 1}},
            {{1, 1, 1}},
            {{0, 1, 1}},
            {{0.5, 0.5, 0}}};

        auto [vertices, tets] = delaunay::delaunay3D(points);
        REQUIRE(vertices.size() == 9);
        REQUIRE(tets.size() == 10);
        validate(vertices, tets);
    }
    SECTION("Cube with point near face")
    {
        std::vector<delaunay::Point3D> points{
            {{0, 0, 0}},
            {{1, 0, 0}},
            {{1, 1, 0}},
            {{0, 1, 0}},
            {{0, 0, 1}},
            {{1, 0, 1}},
            {{1, 1, 1}},
            {{0, 1, 1}},
            {{0.5, 0.5, 1e-12}}};

        auto [vertices, tets] = delaunay::delaunay3D(points);
        REQUIRE(vertices.size() == 9);
        REQUIRE(tets.size() == 12);
        validate(vertices, tets);
    }
    SECTION("Regular grid")
    {
        constexpr size_t N = 10;
        std::vector<delaunay::Point3D> points;
        points.reserve(N * N * N);
        for (size_t i = 0; i < N; i++) {
            for (size_t j = 0; j < N; j++) {
                for (size_t k = 0; k < N; k++) {
                    double x = i;
                    double y = j;
                    double z = k;
                    points.push_back({{x, y, z}});
                }
            }
        }
        auto [vertices, tets] = delaunay::delaunay3D(points);
        REQUIRE(vertices.size() == N * N * N);
        REQUIRE(tets.size() == (N - 1) * (N - 1) * (N - 1) * 6);
        validate(vertices, tets);
    }
}

TEST_CASE("Delaunay2D", "[delaunay][2d]")
{
    using namespace wmtk;

    auto validate = [](const auto& vertices, const auto& triangles) {
        constexpr double EPS = std::numeric_limits<double>::epsilon();
        const size_t num_vertices = vertices.size();
        const size_t num_triangles = triangles.size();

        for (size_t i = 0; i < num_triangles; i++) {
            const auto& tri = triangles[i];
            REQUIRE(tri[0] < num_vertices);
            REQUIRE(tri[1] < num_vertices);
            REQUIRE(tri[2] < num_vertices);

            // tri must be positive oriented and non-degenerate.
            REQUIRE(compute_tri_area(vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]) > EPS);
        }
    };

    SECTION("Simple")
    {
        std::vector<delaunay::Point2D> points{{{0, 0}}, {{1, 0}}, {{0, 1}}, {{1, 1}}};

        auto [vertices, triangles] = delaunay::delaunay2D(points);
        REQUIRE(vertices.size() == 4);
        REQUIRE(triangles.size() == 2);
        validate(vertices, triangles);
    }
    SECTION("Insufficient points should not fail")
    {
        std::vector<delaunay::Point2D> points{{{1, 0}}, {{0, 1}}};

        auto [vertices, triangles] = delaunay::delaunay2D(points);
        REQUIRE(triangles.size() == 0);
        validate(vertices, triangles);
    }
    SECTION("Coplanar pts")
    {
        std::vector<delaunay::Point2D> points{{{0, 0}}, {{1, 0}}, {{2, 0}}, {{3, 0}}};

        auto [vertices, triangles] = delaunay::delaunay2D(points);
        REQUIRE(vertices.size() == 4);
        REQUIRE(triangles.size() == 0);
        validate(vertices, triangles);
    }
    SECTION("Duplicate pts")
    {
        std::vector<delaunay::Point2D>
            points{{{0, 0}}, {{1, 0}}, {{0, 1}}, {{1, 1}}, {{0, 1}}, {{0, 1}}};

        auto [vertices, triangles] = delaunay::delaunay2D(points);
        REQUIRE(vertices.size() == 6); // duplicate pts are kept in the output.
        REQUIRE(triangles.size() == 2);
        validate(vertices, triangles);
    }
    SECTION("Square with centroid")
    {
        std::vector<delaunay::Point2D> points{{{0, 0}}, {{1, 0}}, {{1, 1}}, {{0, 1}}, {{0.5, 0.5}}};

        auto [vertices, triangles] = delaunay::delaunay2D(points);
        REQUIRE(vertices.size() == 5);
        REQUIRE(triangles.size() == 4);
        validate(vertices, triangles);
    }
    SECTION("Regular grid")
    {
        constexpr size_t N = 10;
        std::vector<delaunay::Point2D> points;
        points.reserve(N * N);
        for (size_t i = 0; i < N; i++) {
            for (size_t j = 0; j < N; j++) {
                double x = i;
                double y = j;
                points.push_back({{x, y}});
            }
        }
        auto [vertices, triangles] = delaunay::delaunay2D(points);
        REQUIRE(vertices.size() == N * N);
        REQUIRE(triangles.size() == (N - 1) * (N - 1) * 2);
        validate(vertices, triangles);
    }
}
