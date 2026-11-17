#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/BallRefinement.hpp>

#include <cmath>
#include <limits>
#include <random>

using namespace wmtk::utils;
using Eigen::Vector2d;
using Eigen::Vector3d;

// The circle and sphere tests come from topological_offset (Circle.hpp, Sphere.hpp), whose
// classes moved here.

TEST_CASE("ball_refinement_circle_tri_overlap", "[ball_refinement][2d]")
{
    const RefinementCircle::Cell tri = {{Vector2d(0, 0), Vector2d(0, 1), Vector2d(1, 0)}};
    CHECK(!RefinementCircle{Vector2d(1.0, 1.0), 0.5}.overlaps(tri));
    CHECK(RefinementCircle{Vector2d(0.55, 0.55), 0.5}.overlaps(tri));
    CHECK(RefinementCircle{Vector2d(0.2, 0.2), 0.1}.overlaps(tri));
    CHECK(RefinementCircle{Vector2d(0.2, 0.2), 1000.0}.overlaps(tri));
    CHECK(!RefinementCircle{Vector2d(10.0, 10.0), 1.0}.overlaps(tri));
}

TEST_CASE("ball_refinement_circle_refine", "[ball_refinement][2d]")
{
    const auto children = RefinementCircle{Vector2d(0, 0), 1.0}.refined();
    REQUIRE(children.size() == 4);
    for (const auto& c : children) {
        CHECK(std::abs(c.radius - 0.5) < 1e-12);
        CHECK(std::abs(std::abs(c.center[0]) - 1 / (2 * std::sqrt(2.))) < 1e-12);
        CHECK(std::abs(std::abs(c.center[1]) - 1 / (2 * std::sqrt(2.))) < 1e-12);
    }
    CHECK(children[0].center[0] < 0); // the all-minus child first
    CHECK(children[0].center[1] < 0);
}

TEST_CASE("ball_refinement_circle_bounding", "[ball_refinement][2d]")
{
    const RefinementCircle::Cell tri = {
        {Vector2d(0, 0), Vector2d(1, 0), Vector2d(0.5, 0.5 * std::sqrt(3.0))}};
    const RefinementCircle b = RefinementCircle::bounding(tri);
    CHECK(std::abs(b.radius - std::sqrt(2.0) / 2) < 1e-12);
    for (const auto& p : tri) {
        CHECK((p - b.center).norm() <= b.radius + 1e-12);
    }
}

TEST_CASE("ball_refinement_cube_tet_fit", "[ball_refinement][3d]")
{
    const RefinementSphere::Cell tet = {
        {Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 2, 0), Vector3d(0, 0, 3)}};
    Vector3d c;
    double side;
    fit_box<3>(tet, c, side);
    CHECK((c - Vector3d(0.5, 1.0, 1.5)).norm() < 1e-12);
    CHECK(std::abs(side - 3.0) < 1e-12);
    CHECK(std::abs(RefinementSphere::bounding(tet).radius - 3.0 * std::sqrt(3.0) / 2) < 1e-12);
}

TEST_CASE("ball_refinement_sphere_tet_overlap", "[ball_refinement][3d]")
{
    const RefinementSphere::Cell tet = {
        {Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0), Vector3d(0, 0, 1)}};
    CHECK(!RefinementSphere{Vector3d(1.0, 1.0, 1.0), 0.5}.overlaps(tet));
    CHECK(RefinementSphere{Vector3d(0.55, 0.55, 0.55), 0.5}.overlaps(tet));
    CHECK(RefinementSphere{Vector3d(0.2, 0.2, 0.2), 0.1}.overlaps(tet));
    CHECK(RefinementSphere{Vector3d(0.2, 0.2, 0.2), 1000.0}.overlaps(tet));
    CHECK(!RefinementSphere{Vector3d(10.0, 10.0, 10.0), 1.0}.overlaps(tet));
}

TEST_CASE("ball_refinement_sphere_refine", "[ball_refinement][3d]")
{
    const auto children = RefinementSphere{Vector3d(0, 0, 0), 1.0}.refined();
    REQUIRE(children.size() == 8);
    for (const auto& c : children) {
        CHECK(std::abs(c.radius - 0.5) < 1e-12);
        for (int k = 0; k < 3; ++k) {
            CHECK(std::abs(std::abs(c.center[k]) - 1 / (2 * std::sqrt(3.))) < 1e-12);
        }
    }
    CHECK((children[0].center.array() < 0).all());
}

TEST_CASE("ball_refinement_point_cell_distance", "[ball_refinement]")
{
    const std::array<Vector2d, 3> tri = {{Vector2d(0, 0), Vector2d(1, 0), Vector2d(0, 1)}};
    CHECK(point_cell_distance(Vector2d(0.2, 0.2), tri) == 0.);
    CHECK(std::abs(point_cell_distance(Vector2d(-1, 0.5), tri) - 1) < 1e-12);
    CHECK(std::abs(point_cell_distance(Vector2d(1, 1), tri) - std::sqrt(0.5)) < 1e-12);
    const std::array<Vector3d, 4> tet = {
        {Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0), Vector3d(0, 0, 1)}};
    CHECK(point_cell_distance(Vector3d(0.1, 0.1, 0.1), tet) == 0.);
    CHECK(std::abs(point_cell_distance(Vector3d(0.2, 0.2, -2), tet) - 2) < 1e-12);
    CHECK(std::abs(point_cell_distance(Vector3d(2, 0, 0), tet) - 1) < 1e-12);
}

namespace {

/// Points spread over a cell (corners, edge points and random interior points), for checking a
/// certificate by brute force.
template <int D>
std::vector<Eigen::Matrix<double, D, 1>>
samples(const std::array<Eigen::Matrix<double, D, 1>, D + 1>& cell, std::mt19937& rng, const int n)
{
    std::exponential_distribution<double> E(1.0);
    std::vector<Eigen::Matrix<double, D, 1>> out(cell.begin(), cell.end());
    for (int i = 0; i < n; ++i) {
        std::array<double, D + 1> w;
        double s = 0;
        for (double& x : w) {
            s += (x = E(rng));
        }
        Eigen::Matrix<double, D, 1> p = Eigen::Matrix<double, D, 1>::Zero();
        for (int k = 0; k <= D; ++k) {
            p += w[k] / s * cell[k];
        }
        out.push_back(p);
    }
    return out;
}

/// certify_on_cell against sampling, for f = the distance to a few points, on random cells and
/// levels: whatever it certifies holds at every sample; and it does decide some cells both ways.
template <int D>
void check_certify(const int seed)
{
    using Vec = Eigen::Matrix<double, D, 1>;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U(-1, 1), L(0.05, 1.5);
    int proven_at_most = 0, proven_above = 0;
    for (int trial = 0; trial < 300; ++trial) {
        std::vector<Vec> sites(3);
        for (Vec& s : sites) {
            for (int k = 0; k < D; ++k) s[k] = 2 * U(rng);
        }
        const std::function<double(const Vec&)> f = [&](const Vec& x) {
            double d = std::numeric_limits<double>::infinity();
            for (const Vec& s : sites) d = std::min(d, (x - s).norm());
            return d;
        };
        std::array<Vec, D + 1> cell;
        const double size = L(rng) * 0.5;
        Vec c;
        for (int k = 0; k < D; ++k) c[k] = U(rng);
        for (Vec& p : cell) {
            for (int k = 0; k < D; ++k) p[k] = c[k] + size * U(rng);
        }
        std::array<double, D + 1> corner_values;
        for (int k = 0; k <= D; ++k) corner_values[k] = f(cell[k]);
        const double level = L(rng);
        const auto pts = samples<D>(cell, rng, 400);
        if (certify_on_cell(cell, corner_values, f, level, true, 6)) {
            ++proven_at_most;
            for (const Vec& p : pts) CHECK(f(p) <= level);
        }
        if (certify_on_cell(cell, corner_values, f, level, false, 6)) {
            ++proven_above;
            for (const Vec& p : pts) CHECK(f(p) > level);
        }
    }
    CHECK(proven_at_most > 10);
    CHECK(proven_above > 10);
}

} // namespace

TEST_CASE("ball_refinement_certify_on_cell", "[ball_refinement]")
{
    check_certify<2>(3);
    check_certify<3>(5);

    // NaN refutes, and so does a cell the level crosses
    const std::array<Vector2d, 3> tri = {{Vector2d(0, 0), Vector2d(1, 0), Vector2d(0, 1)}};
    const std::function<double(const Vector2d&)> nan = [](const Vector2d&) {
        return std::numeric_limits<double>::quiet_NaN();
    };
    const double q = std::numeric_limits<double>::quiet_NaN();
    CHECK(!certify_on_cell(tri, {{q, q, q}}, nan, 1.0, true, 4));
    CHECK(!certify_on_cell(tri, {{q, q, q}}, nan, 1.0, false, 4));
    const std::function<double(const Vector2d&)> to_origin = [](const Vector2d& x) {
        return x.norm();
    };
    CHECK(!certify_on_cell(tri, {{0., 1., 1.}}, to_origin, 0.5, true, 8));
    CHECK(!certify_on_cell(tri, {{0., 1., 1.}}, to_origin, 0.5, false, 8));
    CHECK(certify_on_cell(tri, {{0., 1., 1.}}, to_origin, 1.01, true, 8));
}
