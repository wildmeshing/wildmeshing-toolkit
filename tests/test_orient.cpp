#include <catch2/catch_test_macros.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/utils/orient.hpp>

using namespace wmtk;

TEST_CASE("orient2d_double", "[orient][utils]")
{
    const Vector2d p0(0, 0);
    const Vector2d p1(1, 0);
    SECTION("positive")
    {
        const Vector2d p2(0.5, 1e-12);
        CHECK(utils::orient2d(p0, p1, p2));
    }
    SECTION("degenerate")
    {
        const Vector2d p2(0.5, 0);
        CHECK_FALSE(utils::orient2d(p0, p1, p2));
    }
    SECTION("negative")
    {
        const Vector2d p2(0.5, -1e-12);
        CHECK_FALSE(utils::orient2d(p0, p1, p2));
    }
}

TEST_CASE("orient2d_rational", "[orient][utils]")
{
    const Vector2r p0(0, 0);
    const Vector2r p1(1, 0);
    SECTION("positive")
    {
        const Vector2r p2(0.5, 1e-12);
        CHECK(utils::orient2d(p0, p1, p2));
    }
    SECTION("degenerate")
    {
        const Vector2r p2(0.5, 0);
        CHECK_FALSE(utils::orient2d(p0, p1, p2));
    }
    SECTION("negative")
    {
        const Vector2r p2(0.5, -1e-12);
        CHECK_FALSE(utils::orient2d(p0, p1, p2));
    }
}

TEST_CASE("orient3d_double", "[orient][utils]")
{
    const Vector3d p0(0, 0, 0);
    const Vector3d p1(1, 0, 0);
    const Vector3d p2(0.5, 1, 0);
    SECTION("positive")
    {
        const Vector3d p3(0.5, 0.5, 1e-12);
        CHECK(utils::orient3d(p0, p1, p2, p3));
    }
    SECTION("degenerate")
    {
        const Vector3d p3(0.5, 0.5, 0);
        CHECK_FALSE(utils::orient3d(p0, p1, p2, p3));
    }
    SECTION("negative")
    {
        const Vector3d p3(0.5, 0.5, -1e-12);
        CHECK_FALSE(utils::orient3d(p0, p1, p2, p3));
    }
}

TEST_CASE("orient3d_rational", "[orient][utils]")
{
    const Vector3r p0(0, 0, 0);
    const Vector3r p1(1, 0, 0);
    const Vector3r p2(0.5, 1, 0);
    SECTION("positive")
    {
        const Vector3r p3(0.5, 0.5, 1e-12);
        CHECK(utils::orient3d(p0, p1, p2, p3));
    }
    SECTION("degenerate")
    {
        const Vector3r p3(0.5, 0.5, 0);
        CHECK_FALSE(utils::orient3d(p0, p1, p2, p3));
    }
    SECTION("negative")
    {
        const Vector3r p3(0.5, 0.5, -1e-12);
        CHECK_FALSE(utils::orient3d(p0, p1, p2, p3));
    }
}