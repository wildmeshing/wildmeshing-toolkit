#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <wmtk/components/prismatic_mesh/surface_domain.hpp>

using wmtk::MatrixXd;
using wmtk::Vector3d;
using Domain = wmtk::components::prismatic_mesh::internal::SurfaceDomain;
namespace {
MatrixXd points()
{
    MatrixXd p(6, 3);
    p << -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1e-6, 0, 1, 1e-6, 0, 1;
    return p;
}
std::vector<Domain::Triangle> faces()
{
    return {{4, 0, 1}, {4, 1, 5}, {5, 1, 2}, {5, 2, 3}, {5, 3, 4}, {4, 3, 0}};
}
} // namespace

TEST_CASE(
    "Shared offset chart lifts its original vertices and rejects outside points",
    "[surface_domain]")
{
    const auto p = points();
    std::string failure;
    const auto d = Domain::build(p, faces(), {4, 5}, failure);
    REQUIRE(d.has_value());
    CHECK(failure.empty());
    CHECK(d->triangle_count() == 6);
    for (size_t v = 0; v < 6; ++v) {
        const auto lifted = d->position(d->coordinate(v));
        REQUIRE(lifted.has_value());
        CHECK(*lifted == Vector3d(p.row(v)));
    }
    CHECK_FALSE(d->position({3, 3}).has_value());
    CHECK(d->valid({d->coordinate(4), d->coordinate(5)}));
}

TEST_CASE("AMIPS direction pullback respects one-sided chart derivatives", "[surface_domain]")
{
    auto p = points();
    p(0, 2) = 1.3;
    p(2, 2) = .8;
    std::string failure;
    const auto d = Domain::build(p, faces(), {4, 5}, failure);
    REQUIRE(d.has_value());
    const Vector3d gradient(1, .4, -.2);
    const auto origin = d->coordinate(4);
    const auto direction = d->descent(origin, gradient);
    REQUIRE(direction.norm() > 0);
    const auto start = d->position(origin);
    const auto end = d->position(origin + 1e-7 * direction.normalized());
    REQUIRE(start.has_value());
    REQUIRE(end.has_value());
    CHECK(gradient.dot(*end - *start) < 0);
}

TEST_CASE(
    "Domain step ends at an interior edge collapse before the outer boundary",
    "[surface_domain]")
{
    std::string failure;
    const auto d = Domain::build(points(), faces(), {4, 5}, failure);
    REQUIRE(d.has_value());
    const auto a = d->coordinate(4), b = d->coordinate(5);
    const std::array<Domain::UV, 2> direction{b - a, a - b};
    CHECK(std::abs(d->step_limit(direction) - .5) < 1e-12);
    CHECK(d->valid({a + .49 * direction[0], b + .49 * direction[1]}));
    CHECK_FALSE(d->valid({a + .51 * direction[0], b + .51 * direction[1]}));
}

TEST_CASE("Non-disk and nonmanifold offset patches are reported", "[surface_domain]")
{
    MatrixXd p(5, 3);
    p << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, -1, 0;
    std::string failure;
    CHECK_FALSE(Domain::build(p, {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}}, {0, 1}, failure));
    CHECK(failure == "patch_is_not_a_disk");
    CHECK_FALSE(Domain::build(p, {{0, 1, 2}, {0, 3, 1}, {0, 1, 4}}, {0, 1}, failure));
    CHECK(failure == "nonmanifold_edge");
}
