#include <wmtk/utils/AMIPS.h>
#include <wmtk/components/topological_offset/OffsetPotential.hpp>
#include <wmtk/components/topological_offset/SimplicialComplexBVH.hpp>
#include <wmtk/utils/orient.hpp>

#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/solver.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Eigenvalues>

#include <cmath>
#include <random>
#include <set>
#include <utility>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::topological_offset;

/**
 * The calibration gate for the smooth offset potential. The offset is defined as the level set
 * Phi = c, so nothing downstream means anything until these pass. Three questions:
 *   1. Are the derivatives the derivatives? Central finite differences; the analytic expressions
 *      are ipc-toolkit's, so what is really under test is our wrapper around it.
 *   2. Is the calibration self-consistent? Phi = c must recover exactly delta on a straight edge,
 *      since that is how c is defined.
 *   3. How far is the smoothed offset from the Euclidean one? Deliberately not zero -- Phi sums
 *      every feasible primitive, so a reentrant corner gets two barriers where the Euclidean
 *      distance gets one. Pinned on the shapes whose exact offset is known in closed form, so a
 *      change shows up as a number moving rather than as a mesh looking slightly different.
 */

namespace {

constexpr double DHAT_FACTOR = 2.0; // the shipped default: support reaches 2x the offset distance

/// A closed polyline through `pts`, as the (V, E) pair OffsetPotential takes.
struct Polyline
{
    MatrixXd V;
    MatrixXi E;
};

Polyline closed_polyline(const std::vector<Vector2d>& pts)
{
    const int n = static_cast<int>(pts.size());
    Polyline p;
    p.V.resize(n, 2);
    p.E.resize(n, 2);
    for (int i = 0; i < n; ++i) {
        p.V.row(i) = pts[i].transpose();
        p.E(i, 0) = i;
        p.E(i, 1) = (i + 1) % n;
    }
    return p;
}

Polyline open_polyline(const std::vector<Vector2d>& pts)
{
    const int n = static_cast<int>(pts.size());
    Polyline p;
    p.V.resize(n, 2);
    p.E.resize(n - 1, 2);
    for (int i = 0; i < n; ++i) {
        p.V.row(i) = pts[i].transpose();
    }
    for (int i = 0; i + 1 < n; ++i) {
        p.E(i, 0) = i;
        p.E(i, 1) = i + 1;
    }
    return p;
}

/**
 * @brief Points inside the support, in the interior of a single primitive's feasible region.
 *
 * Both halves matter for a finite-difference test: outside the support Phi is identically zero,
 * and within an FD step of a region boundary the two sides of the difference see different active
 * sets. Sampling perpendicular to the middle of each segment gives both.
 */
std::vector<Vector2d> segment_normal_samples(const Polyline& p, const double dhat)
{
    std::vector<Vector2d> out;
    for (int e = 0; e < p.E.rows(); ++e) {
        const Vector2d a = p.V.row(p.E(e, 0)).transpose();
        const Vector2d b = p.V.row(p.E(e, 1)).transpose();
        const Vector2d t = (b - a).normalized();
        const Vector2d n(-t[1], t[0]);
        for (const double u : {0.35, 0.5, 0.65}) {
            const Vector2d m = a + u * (b - a);
            for (const double s : {0.25, 0.5, 0.8}) {
                out.push_back(m + s * dhat * n);
                out.push_back(m - s * dhat * n);
            }
        }
    }
    return out;
}

Polyline circle_polyline(const double R, const int n)
{
    std::vector<Vector2d> pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i) {
        const double a = 2. * M_PI * i / n;
        pts.emplace_back(R * std::cos(a), R * std::sin(a));
    }
    return closed_polyline(pts);
}

/**
 * @brief Where the ray from `origin` in direction `dir` crosses Phi = c, by bisection.
 *
 * The level set is what the offset is, so every geometric comparison below goes through here.
 * Phi decreases monotonically outward along a ray leaving a convex shape, so a sign change
 * brackets a unique crossing.
 */
template <int DIM>
double level_set_radius(
    const OffsetPotential<DIM>& phi,
    // Non-deduced on purpose (DIM comes from `phi` alone), so that call sites may pass an Eigen
    // expression such as VecD::Zero() rather than a materialised vector.
    const typename OffsetPotential<DIM>::VecD& origin,
    const typename OffsetPotential<DIM>::VecD& dir,
    const double lo_in,
    const double hi_in)
{
    const typename OffsetPotential<DIM>::VecD d = dir.normalized();
    double lo = lo_in, hi = hi_in;
    const double c = phi.target_level();
    REQUIRE(phi.value(origin + lo * d) > c); // inside the level set
    REQUIRE(phi.value(origin + hi * d) < c); // outside it
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (phi.value(origin + mid * d) > c) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

// ---------------------------------------------------------------------------------------------
// 3D helpers
// ---------------------------------------------------------------------------------------------

/// A triangle soup as OffsetPotential<3> takes it. `E` is not optional: ipc derives
/// faces_to_edges from it and throws if a face's edge is missing, and an edge's ESP weight counts
/// the triangles on it.
struct TriSoup
{
    MatrixXd V;
    MatrixXi F;
    MatrixXi E;
};

/// Every undirected edge of every triangle, once.
MatrixXi edges_of(const MatrixXi& F)
{
    std::set<std::pair<int, int>> es;
    for (int f = 0; f < F.rows(); ++f) {
        for (int j = 0; j < 3; ++j) {
            const int a = F(f, j), b = F(f, (j + 1) % 3);
            es.emplace(std::min(a, b), std::max(a, b));
        }
    }
    MatrixXi E(es.size(), 2);
    int i = 0;
    for (const auto& [a, b] : es) {
        E(i, 0) = a;
        E(i, 1) = b;
        ++i;
    }
    return E;
}

TriSoup soup(const MatrixXd& V, const MatrixXi& F)
{
    return TriSoup{V, F, edges_of(F)};
}

/// Axis-aligned cube of half-side h, triangulated so that every corner, every edge and every
/// face interior is present -- the three feasible-region kinds 3D has.
TriSoup cube(const double h)
{
    MatrixXd V(8, 3);
    V << -h, -h, -h, h, -h, -h, h, h, -h, -h, h, -h, -h, -h, h, h, -h, h, h, h, h, -h, h, h;
    MatrixXi F(12, 3);
    F << 0, 2, 1, 0, 3, 2, // z = -h
        4, 5, 6, 4, 6, 7, // z = +h
        0, 1, 5, 0, 5, 4, // y = -h
        1, 2, 6, 1, 6, 5, // x = +h
        2, 3, 7, 2, 7, 6, // y = +h
        3, 0, 4, 3, 4, 7; // x = -h
    return soup(V, F);
}

/// UV sphere. Convex, so every point outside is claimed by exactly one primitive and the level
/// set is the Euclidean offset of the polyhedron -- which is what the test measures against.
TriSoup uv_sphere(const double R, const int n_theta, const int n_phi)
{
    std::vector<Vector3d> pts;
    pts.emplace_back(0., 0., R); // north
    for (int i = 1; i < n_theta; ++i) {
        const double th = M_PI * i / n_theta;
        for (int j = 0; j < n_phi; ++j) {
            const double ph = 2. * M_PI * j / n_phi;
            pts.emplace_back(
                R * std::sin(th) * std::cos(ph),
                R * std::sin(th) * std::sin(ph),
                R * std::cos(th));
        }
    }
    pts.emplace_back(0., 0., -R); // south
    const int south = static_cast<int>(pts.size()) - 1;
    const auto ring = [&](int i, int j) { return 1 + (i - 1) * n_phi + (j % n_phi); };

    std::vector<Vector3i> tris;
    for (int j = 0; j < n_phi; ++j) {
        tris.emplace_back(0, ring(1, j), ring(1, j + 1));
        tris.emplace_back(south, ring(n_theta - 1, j + 1), ring(n_theta - 1, j));
    }
    for (int i = 1; i + 1 < n_theta; ++i) {
        for (int j = 0; j < n_phi; ++j) {
            tris.emplace_back(ring(i, j), ring(i + 1, j), ring(i + 1, j + 1));
            tris.emplace_back(ring(i, j), ring(i + 1, j + 1), ring(i, j + 1));
        }
    }

    MatrixXd V(pts.size(), 3);
    for (size_t i = 0; i < pts.size(); ++i) V.row(i) = pts[i].transpose();
    MatrixXi F(tris.size(), 3);
    for (size_t i = 0; i < tris.size(); ++i) F.row(i) = tris[i].transpose();
    return soup(V, F);
}

Vector3d tri_centroid(const TriSoup& s, const int f)
{
    return (s.V.row(s.F(f, 0)) + s.V.row(s.F(f, 1)) + s.V.row(s.F(f, 2))).transpose() / 3.;
}

Vector3d tri_normal(const TriSoup& s, const int f)
{
    const Vector3d a = s.V.row(s.F(f, 0)).transpose();
    const Vector3d b = s.V.row(s.F(f, 1)).transpose();
    const Vector3d c = s.V.row(s.F(f, 2)).transpose();
    return (b - a).cross(c - a).normalized();
}

/// Points inside the support and inside one primitive's feasible region, for finite differences:
/// outside the support there is nothing to differentiate, and near a region boundary the two
/// sides of the difference see different active sets.
std::vector<Vector3d> face_normal_samples(const TriSoup& s, const double dhat)
{
    std::vector<Vector3d> out;
    for (int f = 0; f < s.F.rows(); ++f) {
        const Vector3d c = tri_centroid(s, f);
        const Vector3d n = tri_normal(s, f);
        for (const double t : {0.25, 0.5, 0.8}) {
            out.push_back(c + t * dhat * n);
            out.push_back(c - t * dhat * n);
        }
    }
    return out;
}

} // namespace


TEST_CASE("offset-potential-gradient-fd", "[offset][potential]")
{
    // A polyline with a convex corner, a reentrant corner and two free ends, so the sampled
    // points land in every kind of feasible region there is in 2D.
    const Polyline p = open_polyline(
        {Vector2d(-1., 0.),
         Vector2d(0., 0.),
         Vector2d(0.3, 0.4),
         Vector2d(0.9, 0.1),
         Vector2d(1.6, 0.5)});

    const double delta = 0.1;
    const SmoothOffsetPotential2D phi(p.V, p.E, MatrixXi(0, 3), {}, delta, DHAT_FACTOR);

    const std::vector<Vector2d> samples = segment_normal_samples(p, phi.dhat());
    REQUIRE(samples.size() >= 24);

    const double h = 1e-6;
    for (const Vector2d& x : samples) {
        REQUIRE(phi.value(x) > 0.); // inside the support, or there is nothing to test
        const Vector2d g = phi.gradient(x);
        for (int k = 0; k < 2; ++k) {
            Vector2d dx = Vector2d::Zero();
            dx[k] = h;
            const double fd = (phi.value(x + dx) - phi.value(x - dx)) / (2. * h);
            CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1., std::abs(g[k])));
        }
    }
}


TEST_CASE("offset-potential-hessian-fd", "[offset][potential]")
{
    const Polyline p = open_polyline(
        {Vector2d(-1., 0.), Vector2d(0., 0.), Vector2d(0.3, 0.4), Vector2d(0.9, 0.1)});

    const double delta = 0.1;
    const SmoothOffsetPotential2D phi(p.V, p.E, MatrixXi(0, 3), {}, delta, DHAT_FACTOR);

    const std::vector<Vector2d> samples = segment_normal_samples(p, phi.dhat());
    REQUIRE(samples.size() >= 18);

    const double h = 1e-5;
    for (const Vector2d& x : samples) {
        REQUIRE(phi.value(x) > 0.);
        const Matrix2d H = phi.hessian(x);
        for (int k = 0; k < 2; ++k) {
            Vector2d dx = Vector2d::Zero();
            dx[k] = h;
            const Vector2d fd = (phi.gradient(x + dx) - phi.gradient(x - dx)) / (2. * h);
            for (int j = 0; j < 2; ++j) {
                CHECK(std::abs(fd[j] - H(j, k)) <= 1e-4 * std::max(1., std::abs(H(j, k))));
            }
        }
        CHECK(std::abs(H(0, 1) - H(1, 0)) <= 1e-10 * std::max(1., std::abs(H(0, 1))));
    }
}


TEST_CASE("offset-potential-straight-edge", "[offset][potential]")
{
    // The calibration restated on a different straight edge from the one the constructor
    // calibrates on: c is defined as Phi at distance delta from a long straight edge, so a flat
    // stretch of any input must put the level set at exactly delta.
    const double delta = 0.05;
    const Polyline p = open_polyline({Vector2d(-10., 3.), Vector2d(10., 3.)});
    const SmoothOffsetPotential2D phi(p.V, p.E, MatrixXi(0, 3), {}, delta, DHAT_FACTOR);

    // The closed form the calibration must agree with: one active Vertex2-Edge2P1 pair, so
    // Phi is exactly the normalized clamped-log barrier at the perpendicular distance.
    const double t = delta / phi.dhat();
    const double c_expected = -(t - 1.) * (t - 1.) * std::log(t);
    CHECK(phi.target_level() == Catch::Approx(c_expected).epsilon(1e-12));

    for (const double x : {-4., -1., 0., 2., 5.}) {
        const double r = level_set_radius(
            phi,
            Vector2d(x, 3.),
            Vector2d(0., 1.),
            0.05 * delta,
            0.999 * phi.dhat());
        CHECK(r == Catch::Approx(delta).epsilon(1e-9));
    }

    // ... and the residual is a length that agrees with the true one to first order.
    CHECK(phi.residual_length(Vector2d(0., 3. + delta)) == Catch::Approx(0.).margin(1e-12));
    for (const double e : {0.05 * delta, 0.1 * delta, -0.1 * delta}) {
        const double r = phi.residual_length(Vector2d(0., 3. + delta + e));
        CHECK(r == Catch::Approx(std::abs(e)).epsilon(0.15));
    }
}


TEST_CASE("offset-potential-isolated-point", "[offset][potential]")
{
    // A single isolated vertex: exactly one active Vertex2-Vertex2 pair everywhere, so the
    // level set must be the exact circle of radius delta -- the same closed form the straight
    // edge gives, since both reduce to one barrier evaluated at the Euclidean distance.
    const double delta = 0.2;
    MatrixXd V(1, 2);
    V << 0.4, -0.7;
    const SmoothOffsetPotential2D phi(V, MatrixXi(0, 2), MatrixXi(0, 3), {0}, delta, DHAT_FACTOR);

    const Vector2d o(0.4, -0.7);
    double max_err = 0.;
    for (int i = 0; i < 32; ++i) {
        const double a = 2. * M_PI * i / 32;
        const double r = level_set_radius(
            phi,
            o,
            Vector2d(std::cos(a), std::sin(a)),
            0.05 * delta,
            0.999 * phi.dhat());
        max_err = std::max(max_err, std::abs(r - delta));
    }
    CHECK(max_err <= 1e-9 * delta);
}


TEST_CASE("offset-potential-vs-euclidean-circle", "[offset][potential]")
{
    // A convex closed curve: every point outside is in exactly one primitive's feasible region,
    // so the level set is the Euclidean offset of the polyline. The polyline is inscribed in the
    // circle, so the residual measured here is the polygonal discretisation, not the potential.
    const double R = 1.0;
    const double delta = 0.1;
    const SmoothOffsetPotential2D phi(
        circle_polyline(R, 256).V,
        circle_polyline(R, 256).E,
        MatrixXi(0, 3),
        {},
        delta,
        DHAT_FACTOR);

    double max_err = 0., sum_err = 0.;
    const int n = 64;
    for (int i = 0; i < n; ++i) {
        const double a = 2. * M_PI * (i + 0.37) / n; // off the vertices, deliberately
        const double r = level_set_radius(
            phi,
            Vector2d::Zero(),
            Vector2d(std::cos(a), std::sin(a)),
            R + 0.05 * delta,
            R + 0.999 * phi.dhat());
        const double err = std::abs((r - R) - delta);
        max_err = std::max(max_err, err);
        sum_err += err;
    }
    INFO(
        "circle R=" << R << " delta=" << delta << ": max |offset - delta| = " << max_err << " ("
                    << 100. * max_err / delta << "% of delta), mean "
                    << 100. * (sum_err / n) / delta << "%");
    // 1% of delta. The convex case is the easy one and is expected to be tight.
    CHECK(max_err <= 0.01 * delta);
}


TEST_CASE("offset-potential-vs-euclidean-square", "[offset][potential]")
{
    // Straight sides and convex corners. The exact Euclidean offset of a square is the square
    // grown by delta with quarter-circle corners of radius delta, and both halves are
    // single-primitive regions, so the smoothed offset must reproduce it closely.
    const double h = 1.0; // half-side
    const double delta = 0.1;
    const Polyline p =
        closed_polyline({Vector2d(-h, -h), Vector2d(h, -h), Vector2d(h, h), Vector2d(-h, h)});
    const SmoothOffsetPotential2D phi(p.V, p.E, MatrixXi(0, 3), {}, delta, DHAT_FACTOR);

    // Along the flats: the level set must sit at exactly h + delta.
    double flat_err = 0.;
    for (const double x : {-0.7, -0.3, 0.0, 0.4, 0.8}) {
        const double y = level_set_radius(
            phi,
            Vector2d(x, h),
            Vector2d(0., 1.),
            0.05 * delta,
            0.999 * phi.dhat());
        flat_err = std::max(flat_err, std::abs(y - delta));
    }
    INFO(
        "square flat side: max |offset - delta| = " << flat_err << " (" << 100. * flat_err / delta
                                                    << "% of delta)");
    CHECK(flat_err <= 0.01 * delta);

    // Around a convex corner: the exact offset is the arc of radius delta about the corner.
    double corner_err = 0.;
    // Strictly between +x and +y about the (h,h) corner. The end rays themselves lie on the
    // boundary of a side's feasible region (projection ratio exactly 0 or 1), where ipc's ESP
    // builder and its debug assert classify the point differently and Debug builds abort.
    for (int i = 0; i <= 8; ++i) {
        const double a = 0.5 * M_PI * (i + 0.5) / 9;
        const double r = level_set_radius(
            phi,
            Vector2d(h, h),
            Vector2d(std::cos(a), std::sin(a)),
            0.05 * delta,
            0.999 * phi.dhat());
        corner_err = std::max(corner_err, std::abs(r - delta));
    }
    INFO(
        "square convex corner: max |offset - delta| = "
        << corner_err << " (" << 100. * corner_err / delta << "% of delta)");
    CHECK(corner_err <= 0.01 * delta);
}


TEST_CASE("offset-potential-vs-euclidean-wedge", "[offset][potential]")
{
    // The case where the two offsets genuinely differ. Outside a reentrant corner both adjacent
    // segments claim the point, so Phi is the sum of two barriers where the Euclidean distance is
    // the min of two; Phi decreases outward, so the level set sits further out than delta. That
    // is the smoothing, not an error: it rounds the offset's own concave corner instead of
    // leaving the crease the Euclidean offset has there.
    const double delta = 0.1;
    const double half_angle = M_PI / 4.; // arms 45 degrees either side of +y: a right-angle notch

    // Arms opening upward, so a point on the +y bisector sits in the notch and projects into the
    // interior of both, which is what puts two barriers in the sum. Arms opening the other way
    // give a convex corner, covered by the square above.
    const double arm = 2.;
    const Polyline p = open_polyline(
        {Vector2d(-arm * std::sin(half_angle), arm * std::cos(half_angle)),
         Vector2d(0., 0.),
         Vector2d(arm * std::sin(half_angle), arm * std::cos(half_angle))});
    const SmoothOffsetPotential2D phi(p.V, p.E, MatrixXi(0, 3), {}, delta, DHAT_FACTOR);

    // On the bisector, at height y, the Euclidean distance to the wedge is y*sin(half_angle).
    const double y_bisector =
        level_set_radius(phi, Vector2d::Zero(), Vector2d(0., 1.), 0.05 * delta, 0.999 * phi.dhat());
    const double euclid_bisector = y_bisector * std::sin(half_angle);
    INFO(
        "reentrant wedge, on the bisector: level set at Euclidean distance "
        << euclid_bisector << " vs delta " << delta << " -> "
        << 100. * (euclid_bisector - delta) / delta << "% further out");

    // Pushed outward, never inward. The 1.5 bound documents what summing two nearly equal
    // barriers does; it is not a tuned tolerance.
    CHECK(euclid_bisector > delta);
    CHECK(euclid_bisector <= 1.5 * delta);

    // Far along one arm, only that arm is within the support and the offset is exact again: the
    // smoothing is local to the feature, which is the property that makes it usable.
    const Vector2d dir(std::sin(half_angle), std::cos(half_angle)); // along the +x arm
    const Vector2d nrm(dir[1], -dir[0]); // outward normal of that arm
    const double d_far = level_set_radius(phi, 1.5 * dir, nrm, 0.05 * delta, 0.999 * phi.dhat());
    INFO("reentrant wedge, far along an arm: " << d_far << " vs delta " << delta);
    CHECK(d_far == Catch::Approx(delta).epsilon(0.01));
}


TEST_CASE("offset-potential-support", "[offset][potential]")
{
    const double delta = 0.1;
    const Polyline p = open_polyline({Vector2d(-5., 0.), Vector2d(5., 0.)});
    const SmoothOffsetPotential2D phi(p.V, p.E, MatrixXi(0, 3), {}, delta, DHAT_FACTOR);

    CHECK(phi.dhat() == Catch::Approx(DHAT_FACTOR * delta));

    // Inside the support the potential is positive and the level set is reachable...
    CHECK(phi.within_support(Vector2d(0., 0.5 * delta)));
    CHECK(phi.within_support(Vector2d(0., 1.9 * delta)));
    // ... and beyond it there is nothing at all: no value, no gradient, no direction home.
    // This is exactly the state the runaway guard exists to turn into a hard error.
    CHECK_FALSE(phi.within_support(Vector2d(0., 2.0001 * delta)));
    CHECK(phi.value(Vector2d(0., 3. * delta)) == 0.);
    CHECK(phi.gradient(Vector2d(0., 3. * delta)).norm() == 0.);
    CHECK(phi.hessian(Vector2d(0., 3. * delta)).norm() == 0.);

    // dhat_factor <= 1 puts the offset on (or outside) the support boundary, where the level
    // set does not exist. Refused rather than silently producing a zero field.
    CHECK_THROWS(SmoothOffsetPotential2D(p.V, p.E, MatrixXi(0, 3), {}, delta, 1.0));
}


TEST_CASE("offset-energy-derivatives", "[offset][potential]")
{
    // The smoothing term itself: w (Phi - c)^2, and its gradient, against finite differences.
    const Polyline p = open_polyline(
        {Vector2d(-1., 0.), Vector2d(0., 0.), Vector2d(0.3, 0.4), Vector2d(0.9, 0.1)});
    const double delta = 0.1;
    const auto phi = std::make_shared<const SmoothOffsetPotential2D>(
        p.V,
        p.E,
        MatrixXi(0, 3),
        std::vector<int>{},
        delta,
        DHAT_FACTOR);

    OffsetEnergy2D energy(phi, 1.0);

    const double h = 1e-6;
    for (const Vector2d& x : {Vector2d(-0.5, 0.08), Vector2d(0.15, 0.13), Vector2d(0.62, 0.31)}) {
        VectorXd xv = x;
        VectorXd g;
        energy.gradient(xv, g);
        for (int k = 0; k < 2; ++k) {
            VectorXd xp = xv, xm = xv;
            xp[k] += h;
            xm[k] -= h;
            const double fd = (energy.value(xp) - energy.value(xm)) / (2. * h);
            CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1., std::abs(g[k])));
        }

        // The Gauss-Newton Hessian is the outer product alone, so it is PSD by construction --
        // the whole reason it is the default. The bound must be relative to the matrix scale:
        // the exact zero eigenvalue lands a few ulps of it either side of zero, so an absolute
        // bound fails at small c.
        MatrixXd H;
        energy.hessian(xv, H);
        const Eigen::SelfAdjointEigenSolver<MatrixXd> es(H);
        CHECK(es.eigenvalues().minCoeff() >= -1e-12 * std::max(1., H.norm()));

        // The energy vanishes exactly on the level set, wherever that is along this ray.
        CHECK(energy.value(xv) >= 0.);
    }
}


TEST_CASE("offset-energy-lands-on-the-level-set", "[offset][potential]")
{
    // The end-to-end question: does minimising w (Phi - c)^2 with the solver the smoother uses
    // put a vertex on the level set? Everything above tests the field; this tests that it is
    // usable as an objective. Run without the mesh, so nothing here can be blamed on inversion,
    // the quality veto or the envelope.
    const double delta = 0.25;
    MatrixXd V(1, 2);
    V << 0., 0.;
    const auto phi = std::make_shared<const SmoothOffsetPotential2D>(
        V,
        MatrixXi(0, 2),
        MatrixXi(0, 3),
        std::vector<int>{0},
        delta,
        DHAT_FACTOR);

    auto solver = wmtk::optimization::create_basic_solver();

    // Starting points on both sides of the level set, and one right on it.
    for (const double d0 : {0.5 * delta, 0.75 * delta, 1.0 * delta, 1.3 * delta, 1.8 * delta}) {
        auto energy = std::make_shared<OffsetEnergy2D>(phi, 1.0);
        VectorXd x(2);
        x << d0, 0.;
        try {
            solver->minimize(*energy, x);
        } catch (const std::exception&) {
            // A failed line search is reported by throwing; the position reached is still the
            // best found, exactly as smooth_vertex_2d treats it.
        }
        const double d = x.norm();
        INFO("start " << d0 / delta << "x delta -> " << d / delta << "x delta");
        CHECK(d == Catch::Approx(delta).epsilon(0.02));
    }
}


// =============================================================================================
// 3D. The same three questions, plus one only 3D has: does the broad phase seed all three
// candidate sets? In 2D ipc derives the vertex candidates from the edge candidates; the 3D
// builder reads vf_set, ve_set and vv_set independently and derives nothing, and a missing set is
// silent -- Phi is simply smaller and the level set has a hole at that feature. The cube below is
// the cheapest thing that catches it.
// =============================================================================================


TEST_CASE("offset-potential-3d-calibration", "[offset][potential]")
{
    // c is defined as Phi at perpendicular distance delta from one large flat primitive, and the
    // barrier only ever sees a distance, so the level a 3D run places its offset on must be the
    // same number a 2D run does, and both must be the closed form of the barrier.
    const double delta = 0.05;
    const Polyline seg = open_polyline({Vector2d(-10., 3.), Vector2d(10., 3.)});
    const SmoothOffsetPotential2D phi2(seg.V, seg.E, MatrixXi(0, 3), {}, delta, DHAT_FACTOR);

    const double L = 20.;
    MatrixXd V(3, 3);
    V << -L, -L, 3., L, -L, 3., 0., L, 3.;
    MatrixXi F(1, 3);
    F << 0, 1, 2;
    const SmoothOffsetPotential3D phi3(V, edges_of(F), F, {}, delta, DHAT_FACTOR);

    const double t = delta / phi3.dhat();
    const double c_expected = -(t - 1.) * (t - 1.) * std::log(t);
    CHECK(phi3.target_level() == Catch::Approx(c_expected).epsilon(1e-12));
    CHECK(phi3.target_level() == Catch::Approx(phi2.target_level()).epsilon(1e-12));
    CHECK(phi3.dhat() == Catch::Approx(DHAT_FACTOR * delta));

    // ... and a flat stretch of a different input puts the level set at exactly delta.
    for (const Vector3d& o : {Vector3d(0., 0., 3.), Vector3d(1., -2., 3.), Vector3d(-3., 1., 3.)}) {
        const double r =
            level_set_radius<3>(phi3, o, Vector3d(0., 0., 1.), 0.05 * delta, 0.999 * phi3.dhat());
        CHECK(r == Catch::Approx(delta).epsilon(1e-9));
    }

    // The residual is a length, agreeing with the true one to first order at the level set.
    CHECK(phi3.residual_length(Vector3d(0., 0., 3. + delta)) == Catch::Approx(0.).margin(1e-12));
    for (const double e : {0.05 * delta, 0.1 * delta, -0.1 * delta}) {
        CHECK(
            phi3.residual_length(Vector3d(0., 0., 3. + delta + e)) ==
            Catch::Approx(std::abs(e)).epsilon(0.15));
    }
}


TEST_CASE("offset-potential-3d-gradient-fd", "[offset][potential]")
{
    const double delta = 0.1;
    const TriSoup s = cube(1.0);
    const SmoothOffsetPotential3D phi(s.V, s.E, s.F, {}, delta, DHAT_FACTOR);

    const std::vector<Vector3d> samples = face_normal_samples(s, phi.dhat());
    REQUIRE(samples.size() == 72);

    const double h = 1e-6;
    for (const Vector3d& x : samples) {
        REQUIRE(phi.value(x) > 0.); // inside the support, or there is nothing to test
        const Vector3d g = phi.gradient(x);
        for (int k = 0; k < 3; ++k) {
            Vector3d dx = Vector3d::Zero();
            dx[k] = h;
            const double fd = (phi.value(x + dx) - phi.value(x - dx)) / (2. * h);
            CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1., std::abs(g[k])));
        }
    }
}


TEST_CASE("offset-potential-3d-hessian-fd", "[offset][potential]")
{
    const double delta = 0.1;
    const TriSoup s = cube(1.0);
    const SmoothOffsetPotential3D phi(s.V, s.E, s.F, {}, delta, DHAT_FACTOR);

    const std::vector<Vector3d> samples = face_normal_samples(s, phi.dhat());

    const double h = 1e-5;
    for (const Vector3d& x : samples) {
        REQUIRE(phi.value(x) > 0.);
        const Matrix3d H = phi.hessian(x);
        for (int k = 0; k < 3; ++k) {
            Vector3d dx = Vector3d::Zero();
            dx[k] = h;
            const Vector3d fd = (phi.gradient(x + dx) - phi.gradient(x - dx)) / (2. * h);
            for (int j = 0; j < 3; ++j) {
                CHECK(std::abs(fd[j] - H(j, k)) <= 1e-4 * std::max(1., std::abs(H(j, k))));
            }
        }
        CHECK((H - H.transpose()).norm() <= 1e-10 * std::max(1., H.norm()));
    }
}


TEST_CASE("offset-potential-3d-cube-three-feasible-regions", "[offset][potential]")
{
    // The broad-phase test (see the section note above). Outside a convex cube every point has
    // exactly one closest feature: above a face interior -> that triangle; beside an edge -> that
    // edge; beyond a corner -> that vertex. So the exact Euclidean offset of the cube is also the
    // level set, to machine precision, and a feature the broad phase misses shows up as a CHECK
    // that cannot even bracket the level set.
    const double h = 1.0;
    const double delta = 0.1;
    const TriSoup s = cube(h);
    const SmoothOffsetPotential3D phi(s.V, s.E, s.F, {}, delta, DHAT_FACTOR);

    // Faces. Straight out of every triangle's centroid, which is interior by construction.
    double face_err = 0.;
    for (int f = 0; f < s.F.rows(); ++f) {
        const double r = level_set_radius<3>(
            phi,
            tri_centroid(s, f),
            tri_normal(s, f),
            0.05 * delta,
            0.999 * phi.dhat());
        face_err = std::max(face_err, std::abs(r - delta));
    }
    INFO(
        "cube face: max |offset - delta| = " << face_err << " (" << 100. * face_err / delta
                                             << "% of delta)");
    CHECK(face_err <= 1e-9 * delta);

    // Edges. Out of the middle of each of the 12 cube edges, along the diagonal of the two faces
    // that meet there: the exact offset is the quarter-cylinder of radius delta about the edge.
    double edge_err = 0.;
    for (int axis = 0; axis < 3; ++axis) {
        for (const double sa : {-1., 1.}) {
            for (const double sb : {-1., 1.}) {
                Vector3d o = Vector3d::Zero(), d = Vector3d::Zero();
                o[(axis + 1) % 3] = sa * h;
                o[(axis + 2) % 3] = sb * h;
                d[(axis + 1) % 3] = sa;
                d[(axis + 2) % 3] = sb;
                const double r = level_set_radius<3>(phi, o, d, 0.05 * delta, 0.999 * phi.dhat());
                edge_err = std::max(edge_err, std::abs(r - delta));
            }
        }
    }
    INFO(
        "cube edge: max |offset - delta| = " << edge_err << " (" << 100. * edge_err / delta
                                             << "% of delta)");
    CHECK(edge_err <= 1e-9 * delta);

    // Corners. Out along the body diagonal from each of the 8 corners: the exact offset is the
    // eighth-sphere of radius delta about the corner -- the probe 2D gets for free and 3D does
    // not.
    double corner_err = 0.;
    for (const double sx : {-1., 1.}) {
        for (const double sy : {-1., 1.}) {
            for (const double sz : {-1., 1.}) {
                const Vector3d o(sx * h, sy * h, sz * h);
                const double r = level_set_radius<3>(
                    phi,
                    o,
                    Vector3d(sx, sy, sz),
                    0.05 * delta,
                    0.999 * phi.dhat());
                corner_err = std::max(corner_err, std::abs(r - delta));
            }
        }
    }
    INFO(
        "cube corner: max |offset - delta| = " << corner_err << " (" << 100. * corner_err / delta
                                               << "% of delta)");
    CHECK(corner_err <= 1e-9 * delta);
}


TEST_CASE("offset-potential-3d-vs-euclidean-sphere", "[offset][potential]")
{
    // A convex closed surface, so the level set is the Euclidean offset of the polyhedron, which
    // is inscribed in the sphere -- measuring against the sphere charges the potential for the
    // triangulation. A face's worst point is its centroid, at circumradius^2 / 2R inside the
    // sphere, quadratic in the edge length, so refining must cut the deviation by far more than
    // the 4x asserted below: what is left is the mesh, not Phi.
    const double R = 1.0;
    const double delta = 0.1;

    const auto measure = [&](const int n_theta, const int n_phi) {
        const TriSoup s = uv_sphere(R, n_theta, n_phi);
        const SmoothOffsetPotential3D phi(s.V, s.E, s.F, {}, delta, DHAT_FACTOR);
        double max_err = 0., sum_err = 0.;
        int n = 0;
        for (int i = 1; i < 12; ++i) {
            for (int j = 0; j < 12; ++j) {
                const double th = M_PI * (i + 0.31) / 12.;
                const double ph = 2. * M_PI * (j + 0.17) / 12.; // off the vertices, deliberately
                const Vector3d d(
                    std::sin(th) * std::cos(ph),
                    std::sin(th) * std::sin(ph),
                    std::cos(th));
                const double r = level_set_radius<3>(
                    phi,
                    Vector3d::Zero(),
                    d,
                    R + 0.05 * delta,
                    R + 0.999 * phi.dhat());
                const double err = std::abs((r - R) - delta);
                max_err = std::max(max_err, err);
                sum_err += err;
                ++n;
            }
        }
        return std::make_pair(max_err, sum_err / n);
    };

    const auto [coarse_max, coarse_avg] = measure(24, 48);
    const auto [fine_max, fine_avg] = measure(96, 192);
    INFO(
        "sphere R=" << R << " delta=" << delta << ": max |offset - delta| = " << fine_max << " ("
                    << 100. * fine_max / delta << "% of delta), mean " << 100. * fine_avg / delta
                    << "% | one quarter the edge length: " << 100. * coarse_max / delta << "% max, "
                    << 100. * coarse_avg / delta << "% mean");
    CHECK(fine_max <= 0.01 * delta);
    CHECK(fine_max <= 0.25 * coarse_max); // converging with the mesh, not a floor of Phi's
}


TEST_CASE("offset-potential-3d-wire", "[offset][potential]")
{
    // A 1-dimensional input in 3D: one segment, no triangles, so ESP weighs the segment +1 and
    // its two ends 0. The exact offset is a capsule, and both halves are single-feature regions
    // here: beside the segment its interior is closest, beyond it an endpoint.
    const double delta = 0.1;
    MatrixXd V(2, 3);
    V << -1., 0., 0., 1., 0., 0.;
    MatrixXi E(1, 2);
    E << 0, 1;
    const SmoothOffsetPotential3D phi(V, E, MatrixXi(0, 3), {}, delta, DHAT_FACTOR);

    // The cylindrical part: radially out from points along the segment.
    double cyl_err = 0.;
    for (const double x : {-0.6, -0.2, 0.0, 0.3, 0.7}) {
        for (int i = 0; i < 8; ++i) {
            const double a = 2. * M_PI * i / 8;
            const double r = level_set_radius<3>(
                phi,
                Vector3d(x, 0., 0.),
                Vector3d(0., std::cos(a), std::sin(a)),
                0.05 * delta,
                0.999 * phi.dhat());
            cyl_err = std::max(cyl_err, std::abs(r - delta));
        }
    }
    INFO("wire, cylindrical part: max |offset - delta| = " << cyl_err);
    CHECK(cyl_err <= 1e-9 * delta);

    // The spherical cap beyond an end.
    double cap_err = 0.;
    // Strictly between +x and +y about the (1,0,0) end. The +y ray itself starts on the
    // boundary of the segment's feasible region (projection ratio exactly 1), where ipc's ESP
    // builder and its debug assert classify the point differently and Debug builds abort.
    for (int i = 0; i <= 6; ++i) {
        const double a = 0.5 * M_PI * (i + 0.5) / 7;
        const double r = level_set_radius<3>(
            phi,
            Vector3d(1., 0., 0.),
            Vector3d(std::cos(a), std::sin(a), 0.),
            0.05 * delta,
            0.999 * phi.dhat());
        cap_err = std::max(cap_err, std::abs(r - delta));
    }
    INFO("wire, spherical cap: max |offset - delta| = " << cap_err);
    CHECK(cap_err <= 1e-9 * delta);
}


TEST_CASE("offset-potential-3d-isolated-point", "[offset][potential]")
{
    // A 0-dimensional input in 3D. Besides the geometry, this exercises the sentinel segment:
    // ipc's are_adjacencies_initialized() is false when the mesh has no edges at all, and every
    // accessor the feasible-region test calls then throws.
    const double delta = 0.2;
    MatrixXd V(1, 3);
    V << 0.4, -0.7, 0.2;
    const SmoothOffsetPotential3D phi(V, MatrixXi(0, 2), MatrixXi(0, 3), {0}, delta, DHAT_FACTOR);

    const Vector3d o(0.4, -0.7, 0.2);
    double max_err = 0.;
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            const double th = M_PI * (i + 0.5) / 8., ph = 2. * M_PI * j / 8.;
            const Vector3d d(
                std::sin(th) * std::cos(ph),
                std::sin(th) * std::sin(ph),
                std::cos(th));
            const double r = level_set_radius<3>(phi, o, d, 0.05 * delta, 0.999 * phi.dhat());
            max_err = std::max(max_err, std::abs(r - delta));
        }
    }
    CHECK(max_err <= 1e-9 * delta);
}


TEST_CASE("offset-potential-3d-vs-euclidean-reentrant", "[offset][potential]")
{
    // The case where the two offsets genuinely differ, in 3D: at a right-angle notch between two
    // quads, a point on the bisector projects into the interior of both, so Phi sums two barriers
    // and the level set sits further out than delta. Far from the crease only one quad is within
    // the support and the offset is exact again -- the smoothing is local to the feature.
    const double delta = 0.1;
    const double L = 2.;
    MatrixXd V(6, 3);
    V << 0., -L, 0., L, -L, 0., L, L, 0., 0., L, 0., 0., -L, L, 0., L, L;
    MatrixXi F(4, 3);
    F << 0, 1, 2, 0, 2, 3, // the z = 0 quad
        0, 4, 5, 0, 5, 3; // the x = 0 quad, sharing edge 0-3
    const SmoothOffsetPotential3D phi(V, edges_of(F), F, {}, delta, DHAT_FACTOR);

    const Vector3d bis(1., 0., 1.);
    const double t =
        level_set_radius<3>(phi, Vector3d::Zero(), bis, 0.05 * delta, 0.999 * phi.dhat());
    // On the bisector at parameter t the Euclidean distance to either quad is t/sqrt(2).
    const double euclid = t / std::sqrt(2.);
    INFO(
        "reentrant dihedral, on the bisector: level set at Euclidean distance "
        << euclid << " vs delta " << delta << " -> " << 100. * (euclid - delta) / delta
        << "% further out");
    // Pushed outward, never inward. The 1.5 bound documents what summing two nearly equal
    // barriers does; it is not a tuned tolerance.
    CHECK(euclid > delta);
    CHECK(euclid <= 1.5 * delta);

    // Far from the crease, on the z = 0 quad: exact again.
    const double d_far = level_set_radius<3>(
        phi,
        Vector3d(1.2, 0., 0.),
        Vector3d(0., 0., 1.),
        0.05 * delta,
        0.999 * phi.dhat());
    INFO("reentrant dihedral, far from the crease: " << d_far << " vs delta " << delta);
    CHECK(d_far == Catch::Approx(delta).epsilon(0.01));
}


TEST_CASE("offset-potential-3d-support", "[offset][potential]")
{
    const double delta = 0.1;
    const double L = 5.;
    MatrixXd V(3, 3);
    V << -L, -L, 0., L, -L, 0., 0., L, 0.;
    MatrixXi F(1, 3);
    F << 0, 1, 2;
    const SmoothOffsetPotential3D phi(V, edges_of(F), F, {}, delta, DHAT_FACTOR);

    CHECK(phi.within_support(Vector3d(0., 0., 0.5 * delta)));
    CHECK(phi.within_support(Vector3d(0., 0., 1.9 * delta)));
    // Beyond dhat there is nothing at all: no value, no gradient, no direction home. This is
    // exactly the state the runaway guard exists to turn into a hard error.
    CHECK_FALSE(phi.within_support(Vector3d(0., 0., 2.0001 * delta)));
    CHECK(phi.value(Vector3d(0., 0., 3. * delta)) == 0.);
    CHECK(phi.gradient(Vector3d(0., 0., 3. * delta)).norm() == 0.);
    CHECK(phi.hessian(Vector3d(0., 0., 3. * delta)).norm() == 0.);

    CHECK_THROWS(SmoothOffsetPotential3D(V, edges_of(F), F, {}, delta, 1.0));
    // An edge list that does not cover every triangle edge would silently widen every Voronoi
    // region, so it is refused rather than trusted.
    CHECK_THROWS(SmoothOffsetPotential3D(V, MatrixXi(0, 2), F, {}, delta, DHAT_FACTOR));
}


TEST_CASE("stencil-energy-3d-derivatives", "[offset][potential]")
{
    // The one offset term against finite differences. x enters only through the sample points
    // sliding with it, q_i = a_i x + b_i q1 + c_i q2, so the whole chain rule is the moving
    // vertex's own barycentric weight a_i -- getting that factor wrong is the obvious way to
    // break this, and a corner sample belonging to another vertex (a_i = 0) must contribute to
    // the value while contributing nothing to the gradient.
    const double delta = 0.25;
    MatrixXd V(1, 3);
    V << 0., 0., 0.;
    const auto pot = std::make_shared<const SmoothOffsetPotential3D>(
        V,
        MatrixXi(0, 2),
        MatrixXi(0, 3),
        std::vector<int>{0},
        delta,
        DHAT_FACTOR);

    // Two faces of different shape sharing the moving vertex, and stencils carrying a_i = 1,
    // a_i = 0 and 0 < a_i < 1, so every regime of the chain rule is exercised.
    std::vector<StencilEnergy3D::Face> faces;
    {
        StencilEnergy3D::Face f;
        f.q1 = Vector3d(0.31, -0.05, 0.02);
        f.q2 = Vector3d(0.12, 0.29, -0.04);
        f.samples = {{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}, {1. / 3., 1. / 3., 1. / 3.}};
        faces.push_back(f);
    }
    {
        StencilEnergy3D::Face f;
        f.q1 = Vector3d(0.09, 0.33, 0.11);
        f.q2 = Vector3d(-0.21, 0.17, 0.26);
        f.samples = {{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}, {0.5, 0.25, 0.25}};
        faces.push_back(f);
    }

    const double w = 0.9;
    StencilEnergy3D energy(pot, faces, w);
    VectorXd xv(3);
    xv << 0.21, 0.13, 0.07;

    const double h = 1e-6;
    VectorXd g(3);
    energy.gradient(xv, g);
    for (int k = 0; k < 3; ++k) {
        VectorXd xp = xv, xm = xv;
        xp[k] += h;
        xm[k] -= h;
        const double fd = (energy.value(xp) - energy.value(xm)) / (2. * h);
        INFO("grad k " << k << " fd " << fd << " analytic " << g[k]);
        CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1., std::abs(g[k])));
    }

    // The Gauss-Newton form (gauss_newton = true) is deliberately not the exact Hessian, so it is
    // not checked against central differences of the gradient here; stencil-energy-3d-hessian-fd
    // checks both forms that way. What IS guaranteed, and what the solver needs, is that it is
    // symmetric and PSD by construction, being a sum of outer products.
    StencilEnergy3D energy_gn(pot, faces, w, true);
    MatrixXd H;
    energy_gn.hessian(xv, H);
    CHECK((H - H.transpose()).norm() <= 1e-12 * std::max(1., H.norm()));
    const Eigen::SelfAdjointEigenSolver<MatrixXd> es(H);
    CHECK(es.eigenvalues().minCoeff() >= -1e-12 * std::max(1., H.norm()));
}

TEST_CASE("stencil-energy-2d-derivatives", "[offset][potential]")
{
    // StencilEnergy2D, the 2D front smoother's offset term, against finite differences: the
    // gradient of the value and both Hessian forms of the gradient. As in 3D, x enters only
    // through the samples sliding with it, q_i = a_i x + b_i q1, so the chain rule is the moving
    // vertex's own weight a_i; the stencils carry a_i = 1, a_i = 0 and 0 < a_i < 1. The exact
    // Hessian (the default) must match the difference directly; the Gauss-Newton form must be
    // symmetric PSD and, with the dropped 2 a_i^2 r hess Phi / c term added back from the
    // potential's own hessian(), must recover the exact one. On the smooth field around a point and
    // on the Euclidean field of a segment (via the BVH, as TopoOffsetTriMesh builds it).
    const double delta = 0.25;
    MatrixXd V(1, 2);
    V << 0., 0.;
    const auto smooth = std::make_shared<const SmoothOffsetPotential2D>(
        V,
        MatrixXi(0, 2),
        MatrixXi(0, 3),
        std::vector<int>{0},
        delta,
        DHAT_FACTOR);
    auto bvh = std::make_shared<SimplicialComplexBVH>();
    {
        MatrixXd SV(2, 2);
        SV << -1., 0., 1., 0.;
        MatrixXi SE(1, 2);
        SE << 0, 1;
        bvh->init(SV, MatrixXi(0, 4), MatrixXi(0, 3), SE, MatrixXi(0, 1));
    }
    const auto euclid = std::make_shared<const EuclideanOffsetPotential2D>(bvh, delta);

    struct Case
    {
        const char* name;
        std::shared_ptr<const OffsetPotential2D> pot;
        Eigen::Vector2d x, q1, q2; // chords (x, q1) and (x, q2)
    };
    const std::vector<Case> cases = {
        {"smooth, around a point", smooth, {0.21, 0.13}, {0.31, -0.05}, {0.09, 0.33}},
        {"euclid, above a segment", euclid, {0.1, 0.31}, {0.35, 0.22}, {-0.2, 0.28}}};
    for (const Case& cs : cases) {
        std::vector<StencilEnergy2D::Edge> edges(2);
        edges[0].q1 = cs.q1;
        edges[0].samples = {{1., 0.}, {0., 1.}, {0.5, 0.5}};
        edges[1].q1 = cs.q2;
        edges[1].samples = {{1., 0.}, {0., 1.}, {0.75, 0.25}, {0.25, 0.75}, {0.5, 0.5}};
        const double w = 0.9;
        StencilEnergy2D energy(cs.pot, edges, w);
        VectorXd xv = cs.x;
        INFO(cs.name);

        const double h = 1e-6;
        VectorXd g(2);
        energy.gradient(xv, g);
        MatrixXd H;
        energy.hessian(xv, H);
        for (int k = 0; k < 2; ++k) {
            VectorXd xp = xv, xm = xv;
            xp[k] += h;
            xm[k] -= h;
            const double fd = (energy.value(xp) - energy.value(xm)) / (2. * h);
            CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1., std::abs(g[k])));
            VectorXd gp(2), gm(2);
            energy.gradient(xp, gp);
            energy.gradient(xm, gm);
            for (int j = 0; j < 2; ++j) {
                const double fdh = (gp[j] - gm[j]) / (2. * h);
                CHECK(std::abs(fdh - H(j, k)) <= 1e-4 * std::max(1., H.norm()));
            }
        }

        StencilEnergy2D energy_gn(cs.pot, edges, w, true);
        MatrixXd Hgn;
        energy_gn.hessian(xv, Hgn);
        CHECK((Hgn - Hgn.transpose()).norm() <= 1e-12 * std::max(1., Hgn.norm()));
        const Eigen::SelfAdjointEigenSolver<MatrixXd> es(Hgn);
        CHECK(es.eigenvalues().minCoeff() >= -1e-12 * std::max(1., Hgn.norm()));
        // The dropped term, added back independently: per chord, the mean over its samples of
        // 2 a^2 r hess Phi / c, times the weight.
        const double c = cs.pot->target_level();
        Eigen::Matrix2d dropped = Eigen::Matrix2d::Zero();
        for (const StencilEnergy2D::Edge& e : edges) {
            Eigen::Matrix2d sum = Eigen::Matrix2d::Zero();
            for (const StencilEnergy2D::Sample& sm : e.samples) {
                const Eigen::Vector2d q = sm.a * cs.x + sm.b * e.q1;
                const double r = (cs.pot->value(q) - c) / c;
                sum += (2. * sm.a * sm.a * r / c) * cs.pot->hessian(q);
            }
            dropped += sum / double(e.samples.size());
        }
        CHECK((Hgn + w * dropped - H).norm() <= 1e-9 * std::max(1., H.norm()));
    }
}

TEST_CASE("stencil-energy-3d-hessian-fd", "[offset][potential]")
{
    // Both Hessian forms of StencilEnergy3D against a central difference of the gradient. The
    // exact form (the default) must match it directly. The Gauss-Newton form (gauss_newton =
    // true) drops the 2 r a_i^2 hess Phi / c term on purpose; adding it back here, computed
    // independently from the potential's own hessian(), must recover the exact Hessian to FD
    // precision. That pins two things at once: the kept part is exactly 2 a_i^2 dr dr^T under the
    // per-face mean and the weight, and the dropped part is exactly the one the class comment
    // names -- a Gauss-Newton form returning the exact Hessian, or missing the a_i^2, fails this.
    // Both fields, since the derivatives test above uses the smooth field only and the runs use
    // the Euclidean one; near a face, an edge and a vertex of a cube, since the Euclidean
    // hessian is cased on the feature kind (zero on a face, curved around an edge or a vertex).
    const double delta = 0.1;
    const TriSoup s = cube(1.0);
    const auto smooth = std::make_shared<const SmoothOffsetPotential3D>(
        s.V,
        s.E,
        s.F,
        std::vector<int>{},
        delta,
        DHAT_FACTOR);
    // The exact-kind query envelope, as TopoOffsetTetMesh builds it for the Euclidean field.
    auto env = std::make_shared<SampleEnvelope>();
    env->use_exact = true;
    {
        std::vector<Eigen::Vector3d> verts(size_t(s.V.rows()));
        for (int i = 0; i < s.V.rows(); ++i) verts[size_t(i)] = s.V.row(i).head<3>();
        std::vector<Eigen::Vector3i> tris(size_t(s.F.rows()));
        for (int i = 0; i < s.F.rows(); ++i) {
            tris[size_t(i)] = Eigen::Vector3i(s.F(i, 0), s.F(i, 1), s.F(i, 2));
        }
        env->init(verts, tris, delta);
    }
    const auto euclid = std::make_shared<const EuclideanOffsetPotential3D>(env, delta);

    // Order 0 (the corners) and order 1 (corners + centroid, the default), written by hand as
    // the derivatives test writes its stencils.
    const std::vector<StencilEnergy3D::Sample> order0 = {{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}};
    const std::vector<StencilEnergy3D::Sample> order1 = {
        {1., 0., 0.},
        {0., 1., 0.},
        {0., 0., 1.},
        {1. / 3., 1. / 3., 1. / 3.}};

    // Two faces sharing the moving vertex x, given by their other corners. All samples of a case
    // lie in one feature region, well inside it, so no difference step crosses a region boundary
    // (or, for the smooth field, the support boundary).
    struct Case
    {
        const char* name;
        std::shared_ptr<const OffsetPotential3D> pot;
        Vector3d x, q1, q2, q3; // faces (x, q1, q2) and (x, q2, q3)
        bool flat; // hess Phi == 0 everywhere in the region: the omission must be exactly zero
    };
    const Vector3d fx(0.10, -0.05, 1.08), fq1(0.35, 0.10, 1.11), fq2(0.05, 0.30, 1.13),
        fq3(-0.20, 0.02, 1.06); // above the +z face interior
    const Vector3d ex(1.08, 0.10, 1.06), eq1(1.11, 0.35, 1.09), eq2(1.05, -0.20, 1.12),
        eq3(1.13, -0.05, 1.04); // nearest feature the edge x = z = 1
    const Vector3d vx(1.08, 1.06, 1.07), vq1(1.11, 1.09, 1.03), vq2(1.04, 1.12, 1.08),
        vq3(1.09, 1.02, 1.11); // nearest feature the vertex (1, 1, 1)
    const std::vector<Case> cases = {
        {"smooth, +z face", smooth, fx, fq1, fq2, fq3, false},
        {"euclid, +z face", euclid, fx, fq1, fq2, fq3, true},
        {"euclid, edge region", euclid, ex, eq1, eq2, eq3, false},
        {"euclid, vertex region", euclid, vx, vq1, vq2, vq3, false},
        {"smooth, edge region", smooth, ex, eq1, eq2, eq3, false},
    };

    const double w = 0.9;
    for (const Case& cs : cases) {
        const double c = cs.pot->target_level();
        for (const auto* order : {&order0, &order1}) {
            std::vector<StencilEnergy3D::Face> faces(2);
            faces[0].q1 = cs.q1;
            faces[0].q2 = cs.q2;
            faces[1].q1 = cs.q2;
            faces[1].q2 = cs.q3;
            for (StencilEnergy3D::Face& f : faces) f.samples = *order;
            StencilEnergy3D energy(cs.pot, faces, w);
            const VectorXd xv = cs.x;
            INFO(cs.name << ", " << order->size() << " samples per face");

            // The gradient, as in the derivatives test.
            VectorXd g(3);
            energy.gradient(xv, g);
            for (int k = 0; k < 3; ++k) {
                const double h = 1e-6;
                VectorXd xp = xv, xm = xv;
                xp[k] += h;
                xm[k] -= h;
                const double fd = (energy.value(xp) - energy.value(xm)) / (2. * h);
                INFO("grad k " << k << " fd " << fd << " analytic " << g[k]);
                CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1., std::abs(g[k])));
            }

            // The exact Hessian by central differences of the gradient.
            Matrix3d Hfd = Matrix3d::Zero();
            for (int k = 0; k < 3; ++k) {
                const double h = 1e-5;
                VectorXd xp = xv, xm = xv, gp(3), gm(3);
                xp[k] += h;
                xm[k] -= h;
                energy.gradient(xp, gp);
                energy.gradient(xm, gm);
                Hfd.col(k) = (gp - gm) / (2. * h);
            }
            // The omitted term, from the potential's own Hessian: w sum_f (1/n_f) sum_i
            // 2 r_i a_i^2 hess Phi(q_i) / c.
            Matrix3d dropped = Matrix3d::Zero();
            for (const StencilEnergy3D::Face& f : faces) {
                Matrix3d Df = Matrix3d::Zero();
                for (const StencilEnergy3D::Sample& sm : f.samples) {
                    const Vector3d q = sm.a * cs.x + sm.b * f.q1 + sm.c * f.q2;
                    const double r = (cs.pot->value(q) - c) / c;
                    Df += (2. * r * sm.a * sm.a / c) * cs.pot->hessian(q);
                }
                dropped += Df / double(f.samples.size());
            }
            dropped *= w;

            // The gradient is the same in both forms, so Hfd serves both.
            StencilEnergy3D energy_gn(cs.pot, faces, w, true);
            MatrixXd H, Hgn;
            energy.hessian(xv, H);
            energy_gn.hessian(xv, Hgn);
            const double scale = std::max(1., Hfd.norm());
            INFO(
                "||Hfd - H_exact|| / scale "
                << (Hfd - Matrix3d(H)).norm() / scale << ", ||Hfd - H_gn|| / scale "
                << (Hfd - Matrix3d(Hgn)).norm() / scale << ", ||dropped|| / scale "
                << dropped.norm() / scale);
            // Exact form: the difference itself. No PSD check: the dropped term is indefinite
            // where r < 0, and every sample here happens to be outside the level set.
            CHECK((Hfd - Matrix3d(H)).norm() <= 1e-6 * scale);
            CHECK((H - H.transpose()).norm() <= 1e-12 * std::max(1., H.norm()));
            // Gauss-Newton form: the difference less exactly the dropped term, and PSD.
            CHECK((Hfd - (Matrix3d(Hgn) + dropped)).norm() <= 1e-6 * scale);
            CHECK((Hgn - Hgn.transpose()).norm() <= 1e-12 * std::max(1., Hgn.norm()));
            const Eigen::SelfAdjointEigenSolver<MatrixXd> es(Hgn);
            CHECK(es.eigenvalues().minCoeff() >= -1e-12 * std::max(1., Hgn.norm()));
            if (cs.flat) CHECK(dropped.norm() == 0.);
        }
    }

    // Neither check is vacuous: off the level set in a curved region the omission is a real
    // fraction of the Hessian (measured 25% here), so the Gauss-Newton form does NOT match the
    // finite difference -- the reconstruction above bites on the dropped part, and an exact form
    // that lost the term would fail the direct check above.
    {
        std::vector<StencilEnergy3D::Face> faces(2);
        faces[0].q1 = vq1;
        faces[0].q2 = vq2;
        faces[1].q1 = vq2;
        faces[1].q2 = vq3;
        for (StencilEnergy3D::Face& f : faces) f.samples = order1;
        StencilEnergy3D energy(euclid, faces, w, true);
        const VectorXd xv = vx;
        Matrix3d Hfd = Matrix3d::Zero();
        for (int k = 0; k < 3; ++k) {
            const double h = 1e-5;
            VectorXd xp = xv, xm = xv, gp(3), gm(3);
            xp[k] += h;
            xm[k] -= h;
            energy.gradient(xp, gp);
            energy.gradient(xm, gm);
            Hfd.col(k) = (gp - gm) / (2. * h);
        }
        MatrixXd H;
        energy.hessian(xv, H);
        CHECK((Hfd - Matrix3d(H)).norm() >= 0.1 * Hfd.norm());
    }

    // Where every moving sample sits on the level set the omission vanishes, so the
    // Gauss-Newton Hessian IS the exact one and the finite difference must match the code's
    // Hessian with nothing added -- even though hess Phi != 0 there. Corners on the cylinder of
    // radius delta around the edge x = z = 1, order 0 (the corners alone).
    {
        const auto on_cyl = [&](const double th, const double y) {
            return Vector3d(1. + delta * std::cos(th), y, 1. + delta * std::sin(th));
        };
        std::vector<StencilEnergy3D::Face> faces(2);
        faces[0].q1 = on_cyl(0.4, 0.35);
        faces[0].q2 = on_cyl(1.1, -0.20);
        faces[1].q1 = on_cyl(1.1, -0.20);
        faces[1].q2 = on_cyl(0.9, -0.05);
        for (StencilEnergy3D::Face& f : faces) f.samples = order0;
        StencilEnergy3D energy(euclid, faces, w, true);
        const VectorXd xv = on_cyl(0.7, 0.10);
        REQUIRE(std::abs(euclid->value(Vector3d(xv)) - euclid->target_level()) <= 1e-12);
        Matrix3d Hfd = Matrix3d::Zero();
        for (int k = 0; k < 3; ++k) {
            const double h = 1e-5;
            VectorXd xp = xv, xm = xv, gp(3), gm(3);
            xp[k] += h;
            xm[k] -= h;
            energy.gradient(xp, gp);
            energy.gradient(xm, gm);
            Hfd.col(k) = (gp - gm) / (2. * h);
        }
        MatrixXd H;
        energy.hessian(xv, H);
        CHECK((Hfd - Matrix3d(H)).norm() <= 1e-6 * std::max(1., Hfd.norm()));
    }
}

TEST_CASE("stencil-energy-3d-area-weighted", "[offset][potential]")
{
    // StencilEnergy3D under set_area_weighted(true): n * sum_f area(f) O(f) / sum_f area(f), the
    // areas taken at x. A flat regular hexagon of faces around x in the plane x = 0.1, above the
    // +z face interior of a cube (d = z - 1 there, so r = (d - delta)/delta is affine in z and the
    // whole ring sits inside the level set: r < 0, the front's error cannot reach 0 by moving in
    // the plane -- the pressed front's situation, d growing along it).
    const double delta = 0.1;
    const TriSoup s = cube(1.0);
    auto env = std::make_shared<SampleEnvelope>();
    env->use_exact = true;
    {
        std::vector<Eigen::Vector3d> verts(size_t(s.V.rows()));
        for (int i = 0; i < s.V.rows(); ++i) verts[size_t(i)] = s.V.row(i).head<3>();
        std::vector<Eigen::Vector3i> tris(size_t(s.F.rows()));
        for (int i = 0; i < s.F.rows(); ++i) {
            tris[size_t(i)] = Eigen::Vector3i(s.F(i, 0), s.F(i, 1), s.F(i, 2));
        }
        env->init(verts, tris, delta);
    }
    const auto pot = std::make_shared<const EuclideanOffsetPotential3D>(env, delta);
    const std::vector<StencilEnergy3D::Sample> order1 = {
        {1., 0., 0.},
        {0., 1., 0.},
        {0., 0., 1.},
        {1. / 3., 1. / 3., 1. / 3.}};
    const double w = 0.9, h = 0.02;
    const auto ring = [&](const Vector3d& c) {
        std::vector<StencilEnergy3D::Face> faces(6);
        for (int k = 0; k < 6; ++k) {
            const double t0 = M_PI / 3. * k, t1 = M_PI / 3. * (k + 1);
            faces[size_t(k)].q1 = c + h * Vector3d(0., std::cos(t0), std::sin(t0));
            faces[size_t(k)].q2 = c + h * Vector3d(0., std::cos(t1), std::sin(t1));
            faces[size_t(k)].samples = order1;
        }
        return faces;
    };
    const Vector3d c0(0.1, 0.0, 1.05);

    SECTION("derivatives")
    {
        StencilEnergy3D e(pot, ring(c0), w);
        e.set_area_weighted(true);
        const VectorXd xv = c0 + Vector3d(0.002, 0.003, 0.004);
        VectorXd g(3);
        e.gradient(xv, g);
        for (int k = 0; k < 3; ++k) {
            const double dh = 1e-6;
            VectorXd xp = xv, xm = xv;
            xp[k] += dh;
            xm[k] -= dh;
            const double fd = (e.value(xp) - e.value(xm)) / (2. * dh);
            INFO("grad k " << k << " fd " << fd << " analytic " << g[k]);
            CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1., std::abs(g[k])));
        }
        MatrixXd H(3, 3);
        e.hessian(xv, H);
        for (int k = 0; k < 3; ++k) {
            const double dh = 1e-5;
            VectorXd xp = xv, xm = xv, gp(3), gm(3);
            xp[k] += dh;
            xm[k] -= dh;
            e.gradient(xp, gp);
            e.gradient(xm, gm);
            const VectorXd col = (gp - gm) / (2. * dh);
            for (int j = 0; j < 3; ++j) {
                INFO("hess " << j << "," << k << " fd " << col[j] << " analytic " << H(j, k));
                CHECK(std::abs(col[j] - H(j, k)) <= 1e-4 * std::max(1., std::abs(H(j, k))));
            }
        }
    }

    SECTION("equal areas: the plain sum")
    {
        StencilEnergy3D plain(pot, ring(c0), w), weighted(pot, ring(c0), w);
        weighted.set_area_weighted(true);
        const VectorXd xv = c0;
        CHECK(weighted.value(xv) == Catch::Approx(plain.value(xv)).epsilon(1e-12));
    }

    SECTION("no slide at the centre")
    {
        // The plain sum slides x toward larger z (r -> 0); the area-weighted one does not move
        // it within the plane: the offset part of r is integrated exactly and r^2's quadratic
        // part is symmetric about the centre.
        StencilEnergy3D plain(pot, ring(c0), w), weighted(pot, ring(c0), w);
        weighted.set_area_weighted(true);
        const VectorXd xv = c0;
        VectorXd gp(3), gw(3);
        plain.gradient(xv, gp);
        weighted.gradient(xv, gw);
        INFO("plain " << gp.transpose() << " | weighted " << gw.transpose());
        CHECK(gp[2] < 0.);
        CHECK(std::abs(gw[1]) <= 1e-10 * std::abs(gp[2]));
        CHECK(std::abs(gw[2]) <= 1e-10 * std::abs(gp[2]));
    }

    SECTION("the slide does not depend on the error's offset")
    {
        // The same ring and the same off-centre x, at two heights: r differs by a constant.
        // The plain sum's sliding gradient changes with it; the area-weighted one does not --
        // what is left of it is the stencil's quadrature error on r^2's quadratic part.
        const Vector3d off(0., 0.004, 0.006), lift(0., 0., 0.01);
        StencilEnergy3D pa(pot, ring(c0), w), pb(pot, ring(c0 + lift), w);
        StencilEnergy3D wa(pot, ring(c0), w), wb(pot, ring(c0 + lift), w);
        wa.set_area_weighted(true);
        wb.set_area_weighted(true);
        VectorXd gpa(3), gpb(3), gwa(3), gwb(3);
        pa.gradient(VectorXd(c0 + off), gpa);
        pb.gradient(VectorXd(c0 + lift + off), gpb);
        wa.gradient(VectorXd(c0 + off), gwa);
        wb.gradient(VectorXd(c0 + lift + off), gwb);
        INFO(
            "plain " << gpa.transpose() << " / " << gpb.transpose() << " | weighted "
                     << gwa.transpose() << " / " << gwb.transpose());
        CHECK(std::abs(gpa[2] - gpb[2]) > 0.1 * std::abs(gpa[2]));
        for (int k : {1, 2}) {
            CHECK(std::abs(gwa[k] - gwb[k]) <= 1e-9 * std::max(std::abs(gpa[2]), 1.));
        }
        // and what is left is small beside the plain sum's slide
        CHECK(std::abs(gwa[2]) < 0.1 * std::abs(gpa[2]));
    }
    SECTION("integral mode: derivatives, the plain sum times the area, no slide at the centre")
    {
        // EXPERIMENTAL_integral_energy's front term: weight * sum_f area(f) mean_f(r^2), no
        // division by the ring's area.
        StencilEnergy3D e(pot, ring(c0), w), plain(pot, ring(c0), w);
        e.set_area_integral(true);
        const VectorXd xv = c0 + Vector3d(0.002, 0.003, 0.004);
        VectorXd g(3);
        e.gradient(xv, g);
        MatrixXd H(3, 3);
        e.hessian(xv, H);
        for (int k = 0; k < 3; ++k) {
            const double dh = 1e-6;
            VectorXd xp = xv, xm = xv;
            xp[k] += dh;
            xm[k] -= dh;
            const double fd = (e.value(xp) - e.value(xm)) / (2. * dh);
            INFO("grad k " << k << " fd " << fd << " analytic " << g[k]);
            CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1e-3, std::abs(g[k])));
            VectorXd gp(3), gm(3);
            const double dh2 = 1e-5;
            VectorXd yp = xv, ym = xv;
            yp[k] += dh2;
            ym[k] -= dh2;
            e.gradient(yp, gp);
            e.gradient(ym, gm);
            for (int j = 0; j < 3; ++j) {
                const double fdh = (gp[j] - gm[j]) / (2. * dh2);
                INFO("hess " << j << "," << k << " fd " << fdh << " analytic " << H(j, k));
                CHECK(std::abs(fdh - H(j, k)) <= 1e-4 * std::max(1e-3, std::abs(H(j, k))));
            }
        }
        // At the centre every face has the same area A: the sum is A times the plain sum.
        const double A = 0.5 * h * h * std::sin(M_PI / 3.);
        CHECK(e.value(VectorXd(c0)) == Catch::Approx(A * plain.value(VectorXd(c0))).epsilon(1e-12));
        VectorXd gc(3), gpl(3);
        e.gradient(VectorXd(c0), gc);
        plain.gradient(VectorXd(c0), gpl);
        CHECK(std::abs(gc[1]) <= 1e-10 * A * std::abs(gpl[2]));
        CHECK(std::abs(gc[2]) <= 1e-10 * A * std::abs(gpl[2]));
    }

    SECTION("with the quadratic stencil weights, no slide anywhere")
    {
        // Corners 1, centroid 9 (corners 1/12, centroid 3/4): O(f) is then the face's exact mean
        // of e^2 for this affine e, so the ring's area-weighted mean is the exact mean of e^2 over
        // the fixed hexagon and sliding x within it changes nothing -- off the centre too.
        auto qring = [&](const Vector3d& c) {
            auto faces = ring(c);
            for (auto& fc : faces) {
                for (auto& sm : fc.samples) sm.w = (sm.a == 1. / 3.) ? 9. : 1.;
            }
            return faces;
        };
        const Vector3d off(0., 0.004, 0.006);
        StencilEnergy3D plain(pot, ring(c0), w), wq(pot, qring(c0), w);
        wq.set_area_weighted(true);
        VectorXd gp(3), gq(3);
        plain.gradient(VectorXd(c0 + off), gp);
        wq.gradient(VectorXd(c0 + off), gq);
        INFO("plain " << gp.transpose() << " | area + quadratic weights " << gq.transpose());
        CHECK(std::abs(gq[1]) <= 1e-9 * std::abs(gp[2]));
        CHECK(std::abs(gq[2]) <= 1e-9 * std::abs(gp[2]));
    }
}

TEST_CASE("stencil-energy-3d-is-the-mean-squared-relative-error", "[offset][potential]")
{
    // The value read back from the formula independently: w * sum over faces of the MEAN over
    // that face's samples of ((Phi(q) - c)/c)^2. Mean WITHIN a face, sum ACROSS faces, and no
    // area weighting -- the three structural choices the instruction specified, each pinned
    // below by a property that fails if it is a sum within a face or a mean across them.
    const double delta = 0.25;
    MatrixXd V(1, 3);
    V << 0., 0., 0.;
    const auto pot = std::make_shared<const SmoothOffsetPotential3D>(
        V,
        MatrixXi(0, 2),
        MatrixXi(0, 3),
        std::vector<int>{0},
        delta,
        DHAT_FACTOR);

    std::vector<StencilEnergy3D::Face> faces;
    {
        StencilEnergy3D::Face f;
        f.q1 = Vector3d(0.31, -0.05, 0.02);
        f.q2 = Vector3d(0.12, 0.29, -0.04);
        f.samples = {{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}, {1. / 3., 1. / 3., 1. / 3.}};
        faces.push_back(f);
    }
    {
        StencilEnergy3D::Face f;
        f.q1 = Vector3d(0.09, 0.33, 0.11);
        f.q2 = Vector3d(-0.21, 0.17, 0.26);
        f.samples = {{0.5, 0.25, 0.25}, {0.25, 0.5, 0.25}};
        faces.push_back(f);
    }

    const double w = 0.7;
    const Vector3d x(0.21, 0.13, 0.07);
    VectorXd xv = x;
    const double c = pot->target_level();

    double expect = 0.;
    for (const StencilEnergy3D::Face& f : faces) {
        double s = 0.;
        for (const StencilEnergy3D::Sample& sm : f.samples) {
            const Vector3d q = sm.a * x + sm.b * f.q1 + sm.c * f.q2;
            const double r = (pot->value(q) - c) / c;
            s += r * r;
        }
        expect += s / double(f.samples.size());
    }
    expect *= w;

    StencilEnergy3D energy(pot, faces, w);
    const double got = energy.value(xv);
    CHECK(got == Catch::Approx(expect).epsilon(1e-12));

    // MEAN WITHIN A FACE: repeating a face's samples leaves its contribution unchanged. A sum
    // within the face would double it.
    {
        std::vector<StencilEnergy3D::Face> dup = faces;
        for (StencilEnergy3D::Face& f : dup) {
            const auto once = f.samples;
            f.samples.insert(f.samples.end(), once.begin(), once.end());
        }
        StencilEnergy3D e2(pot, dup, w);
        CHECK(e2.value(xv) == Catch::Approx(got).epsilon(1e-12));
    }

    // SUM ACROSS FACES: listing the same faces twice doubles the energy. A mean across faces
    // would leave it unchanged.
    {
        std::vector<StencilEnergy3D::Face> twice = faces;
        twice.insert(twice.end(), faces.begin(), faces.end());
        StencilEnergy3D e3(pot, twice, w);
        CHECK(e3.value(xv) == Catch::Approx(2. * got).epsilon(1e-12));
    }

    // NO AREA WEIGHTING: scaling a face's two fixed corners away from the moving vertex changes
    // its area but, with the same barycentric weights, moves the sample points too -- so the
    // check that bites is the simpler one, that the weight is the only prefactor.
    {
        StencilEnergy3D e4(pot, faces, 2. * w);
        CHECK(e4.value(xv) == Catch::Approx(2. * got).epsilon(1e-12));
    }

    // A sample that sits exactly on the level set contributes nothing, whatever the field: its
    // relative error is zero by construction. residual_length() locates the level set here.
    {
        std::vector<StencilEnergy3D::Face> on_level;
        StencilEnergy3D::Face f;
        f.q1 = Vector3d(1., 0., 0.);
        f.q2 = Vector3d(0., 1., 0.);
        f.samples = {{1., 0., 0.}}; // the moving vertex alone
        on_level.push_back(f);
        StencilEnergy3D e5(pot, on_level, w);
        // Bisect along +z for the radius where Phi == c.
        double lo = 0.5 * delta, hi = 3. * delta;
        for (int i = 0; i < 200; ++i) {
            const double mid = 0.5 * (lo + hi);
            (pot->value(Vector3d(0., 0., mid)) > c ? lo : hi) = mid;
        }
        VectorXd on(3);
        on << 0., 0., 0.5 * (lo + hi);
        CHECK(std::abs(pot->value(Vector3d(on)) - c) <= 1e-12 * c);
        CHECK(e5.value(on) == Catch::Approx(0.).margin(1e-20));
        VectorXd gz(3);
        e5.gradient(on, gz);
        CHECK(gz.norm() == Catch::Approx(0.).margin(1e-12));
    }
}

TEST_CASE("cubed-amips-energy-3d-volume-weighted", "[offset][potential]")
{
    // EXPERIMENTAL_integral_energy's AMIPS part: w sum_t vol(t) AMIPS(t)^3 with the moving vertex
    // first in each cell. The value against vol and AMIPS computed independently, gradient and
    // Hessian against finite differences, and a cell volume that cancels in the ring: for equal
    // AMIPS the ring's volumes sum to a constant, so only the shapes pull.
    std::vector<std::array<double, 12>> cells;
    const std::array<std::array<Vector3d, 3>, 3> others = {{
        {{Vector3d(1., 0.1, 0.), Vector3d(0.2, 1.1, 0.05), Vector3d(0.1, 0.3, 0.9)}},
        {{Vector3d(-0.9, 0.2, 0.1), Vector3d(-0.1, -0.8, 0.3), Vector3d(0.1, 0.1, -1.)}},
        {{Vector3d(0.3, -1., 0.2), Vector3d(1., 0.2, -0.4), Vector3d(-0.2, 0.1, 1.1)}},
    }};
    for (auto o : others) {
        if (!wmtk::utils::orient3d(Vector3d::Zero(), o[0], o[1], o[2])) std::swap(o[1], o[2]);
        std::array<double, 12> c{};
        for (int k = 0; k < 3; ++k) {
            for (int j = 0; j < 3; ++j) c[size_t(3 + 3 * k + j)] = o[size_t(k)][j];
        }
        cells.push_back(c);
    }
    const double w = 0.7;
    CubedAMIPSEnergy3D energy(cells, w);
    energy.set_volume_weighted(true);
    const double h = 1e-6;
    for (const Vector3d& x :
         {Vector3d(0.05, -0.02, 0.03), Vector3d(-0.1, 0.1, 0.), Vector3d(0., 0., 0.)}) {
        VectorXd xv = x;
        double want = 0.;
        for (auto c : cells) {
            c[0] = x[0];
            c[1] = x[1];
            c[2] = x[2];
            const Vector3d p0(c[0], c[1], c[2]), p1(c[3], c[4], c[5]), p2(c[6], c[7], c[8]),
                p3(c[9], c[10], c[11]);
            const double vol = std::abs((p1 - p0).dot((p2 - p0).cross(p3 - p0))) / 6.;
            const double a = wmtk::AMIPS_energy(c);
            want += vol * a * a * a;
        }
        CHECK(energy.value(xv) == Catch::Approx(w * want).epsilon(1e-12));
        VectorXd g;
        energy.gradient(xv, g);
        MatrixXd H;
        energy.hessian(xv, H);
        for (int k = 0; k < 3; ++k) {
            VectorXd xp = xv, xm = xv;
            xp[k] += h;
            xm[k] -= h;
            const double fd = (energy.value(xp) - energy.value(xm)) / (2. * h);
            INFO("k " << k << " fd " << fd << " analytic " << g[k]);
            CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1., std::abs(g[k])));
            VectorXd gp, gm;
            energy.gradient(xp, gp);
            energy.gradient(xm, gm);
            for (int j = 0; j < 3; ++j) {
                const double fdh = (gp[j] - gm[j]) / (2. * h);
                INFO("H(" << j << "," << k << ") fd " << fdh << " analytic " << H(j, k));
                CHECK(std::abs(fdh - H(j, k)) <= 1e-4 * std::max(1., std::abs(H(j, k))));
            }
        }
    }
}

TEST_CASE("band-volume-energy-3d", "[offset][potential]")
{
    // EXPERIMENTAL_band_volume_energy's front term: w sum_t Vol_t (mean of r over t's corners and
    // centroid). Cells are the octants of an octahedron around x (corners center +- rad e_i), all
    // eight (a vertex inside the band) or the four with z below (a front vertex). Near a cube of
    // half-size 1: above the +z face interior d = z - 1 is affine; beyond the edge x = z = 1 it is
    // the distance to that edge, curved.
    const double delta = 0.1;
    const TriSoup s = cube(1.0);
    auto env = std::make_shared<SampleEnvelope>();
    env->use_exact = true;
    {
        std::vector<Eigen::Vector3d> verts(size_t(s.V.rows()));
        for (int i = 0; i < s.V.rows(); ++i) verts[size_t(i)] = s.V.row(i).head<3>();
        std::vector<Eigen::Vector3i> tris(size_t(s.F.rows()));
        for (int i = 0; i < s.F.rows(); ++i) {
            tris[size_t(i)] = Eigen::Vector3i(s.F(i, 0), s.F(i, 1), s.F(i, 2));
        }
        env->init(verts, tris, delta);
    }
    const auto pot = std::make_shared<const EuclideanOffsetPotential3D>(env, delta);
    const double c = pot->target_level();
    const auto octants = [](const Vector3d& ctr, const double rad, const bool lower_only) {
        std::vector<BandVolumeEnergy3D::Cell> cells;
        for (const double sx : {-1., 1.}) {
            for (const double sy : {-1., 1.}) {
                for (const double sz : {-1., 1.}) {
                    if (lower_only && sz > 0.) continue;
                    BandVolumeEnergy3D::Cell cl;
                    cl.q1 = ctr + sx * rad * Vector3d::UnitX();
                    cl.q2 = ctr + sy * rad * Vector3d::UnitY();
                    cl.q3 = ctr + sz * rad * Vector3d::UnitZ();
                    cells.push_back(cl);
                }
            }
        }
        return cells;
    };
    const double w = 0.8;

    SECTION("value: volume times the five-point mean, which is the centroid value on affine d")
    {
        const Vector3d ctr(0.1, 0.2, 1.3);
        const auto cells = octants(ctr, 0.05, false);
        BandVolumeEnergy3D energy(pot, cells, ctr, w);
        const Vector3d x = ctr + Vector3d(0.01, -0.02, 0.015);
        double want = 0., want_centroid = 0.;
        for (const auto& cl : cells) {
            const double vol = std::abs((cl.q1 - x).dot((cl.q2 - x).cross(cl.q3 - x))) / 6.;
            const Vector3d ctd = 0.25 * (x + cl.q1 + cl.q2 + cl.q3);
            double m = 0.;
            for (const Vector3d& q : {x, cl.q1, cl.q2, cl.q3, ctd}) m += (pot->value(q) - c) / c;
            want += vol * m / 5.;
            want_centroid += vol * (pot->value(ctd) - c) / c;
        }
        const VectorXd xv = x;
        CHECK(energy.value(xv) == Catch::Approx(w * want).epsilon(1e-12));
        CHECK(energy.value(xv) == Catch::Approx(w * want_centroid).epsilon(1e-10));
        energy.set_centroid_only(true);
        CHECK(energy.value(xv) == Catch::Approx(w * want_centroid).epsilon(1e-12));
    }

    SECTION("a vertex inside the band feels nothing where d is affine")
    {
        // The eight cells tile a fixed octahedron wherever x is inside it, and the rule is exact
        // on affine d: the sum is the integral over the octahedron, independent of x.
        const Vector3d ctr(0.1, 0.2, 1.3);
        BandVolumeEnergy3D energy(pot, octants(ctr, 0.05, false), ctr, w);
        const VectorXd x0 = ctr, x1 = ctr + Vector3d(0.012, -0.007, 0.02);
        CHECK(energy.value(x1) == Catch::Approx(energy.value(x0)).epsilon(1e-10));
        VectorXd g;
        energy.gradient(x1, g);
        CHECK(g.norm() <= 1e-10 * std::max(1., std::abs(energy.value(x1)) / 0.05));
    }

    SECTION("gradient and Hessian against finite differences, affine and curved d")
    {
        struct Case
        {
            const char* name;
            Vector3d ctr;
            double rad;
            bool lower_only;
            Vector3d dx;
        };
        const std::vector<Case> cases = {
            {"front vertex over a face",
             Vector3d(0.1, 0.2, 1.3),
             0.05,
             true,
             Vector3d(0.004, -0.003, 0.006)},
            {"front vertex beyond an edge",
             Vector3d(1.06, 0.0, 1.07),
             0.03,
             true,
             Vector3d(0.002, 0.001, -0.003)},
            {"band vertex beyond an edge",
             Vector3d(1.06, 0.0, 1.07),
             0.03,
             false,
             Vector3d(-0.002, 0.003, 0.001)},
        };
        const double h = 1e-6;
        for (const Case& cs : cases)
            for (const bool centroid_only : {false, true}) {
                INFO(cs.name << (centroid_only ? ", centroid only" : ", five points"));
                BandVolumeEnergy3D energy(pot, octants(cs.ctr, cs.rad, cs.lower_only), cs.ctr, w);
                energy.set_centroid_only(centroid_only);
                const VectorXd xv = cs.ctr + cs.dx;
                VectorXd g;
                energy.gradient(xv, g);
                MatrixXd H;
                energy.hessian(xv, H);
                const double gs = std::max(1e-6, g.norm()), hs = std::max(1e-6, H.norm());
                for (int k = 0; k < 3; ++k) {
                    VectorXd xp = xv, xm = xv;
                    xp[k] += h;
                    xm[k] -= h;
                    const double fd = (energy.value(xp) - energy.value(xm)) / (2. * h);
                    INFO("k " << k << " fd " << fd << " analytic " << g[k]);
                    CHECK(std::abs(fd - g[k]) <= 1e-6 * gs);
                    VectorXd gp, gm;
                    energy.gradient(xp, gp);
                    energy.gradient(xm, gm);
                    for (int j = 0; j < 3; ++j) {
                        const double fdh = (gp[j] - gm[j]) / (2. * h);
                        INFO("H(" << j << "," << k << ") fd " << fdh << " analytic " << H(j, k));
                        CHECK(std::abs(fdh - H(j, k)) <= 1e-5 * hs);
                    }
                }
            }
    }
}

TEST_CASE("band-volume-corner-bound", "[offset][potential]")
{
    // EXPERIMENTAL_band_volume_rule "corner_bound" on a cube of half-size 1: InputTriangles'
    // nearest triangle and per-triangle distance; the rule Vol * min_P corner mean of
    // (d_P - delta)/delta is an upper bound on a dense integral; a split never raises it when the
    // children may use the parent's minimiser; BandVolumeEnergy3D's corner-bound derivatives.
    const double delta = 0.1;
    const TriSoup s = cube(1.0);
    Eigen::MatrixXd V(s.V.rows(), 3);
    for (int i = 0; i < s.V.rows(); ++i) V.row(i) = s.V.row(i).head<3>();
    Eigen::MatrixXi F = s.F;
    const auto tris = std::make_shared<const InputTriangles>(V, F);
    const auto d_all = [&](const Vector3d& p) {
        double m = std::numeric_limits<double>::infinity();
        for (size_t t = 0; t < tris->size(); ++t) m = std::min(m, tris->distance(int64_t(t), p));
        return m;
    };
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> U(-1.4, 1.4);

    SECTION("nearest triangle and the distance's derivatives")
    {
        for (int k = 0; k < 200; ++k) {
            const Vector3d p(U(rng), U(rng), U(rng));
            const int64_t t = tris->nearest(p);
            CHECK(tris->distance(t, p) == Catch::Approx(d_all(p)).margin(1e-12));
        }
        const double h = 1e-6;
        // a face region, an edge region and a vertex region of triangle-nearest features
        for (const Vector3d& p :
             {Vector3d(0.2, 0.3, 1.25), Vector3d(1.2, 0.1, 1.15), Vector3d(1.1, 1.2, 1.3)}) {
            const int64_t t = tris->nearest(p);
            Vector3d g;
            Eigen::Matrix3d H;
            tris->distance(t, p, &g, &H);
            for (int j = 0; j < 3; ++j) {
                Vector3d pp = p, pm = p;
                pp[j] += h;
                pm[j] -= h;
                CHECK(
                    std::abs((tris->distance(t, pp) - tris->distance(t, pm)) / (2 * h) - g[j]) <=
                    1e-6);
                Vector3d gp, gm;
                tris->distance(t, pp, &gp);
                tris->distance(t, pm, &gm);
                for (int i = 0; i < 3; ++i)
                    CHECK(std::abs((gp[i] - gm[i]) / (2 * h) - H(i, j)) <= 1e-5);
            }
        }
    }

    // The rule on four corners with an optional extra candidate; returns the value and the
    // minimiser.
    const auto rule = [&](const std::array<Vector3d, 4>& q, const int64_t extra, int64_t& best) {
        const double vol = std::abs((q[1] - q[0]).dot((q[2] - q[0]).cross(q[3] - q[0]))) / 6.;
        std::vector<int64_t> cand = {extra};
        for (const Vector3d& p : q) cand.push_back(tris->nearest(p));
        double m = std::numeric_limits<double>::infinity();
        best = -1;
        for (const int64_t P : cand) {
            if (P < 0) continue;
            double sum = 0.;
            for (const Vector3d& p : q) sum += (tris->distance(P, p) - delta) / delta;
            if (sum / 4. < m) m = sum / 4., best = P;
        }
        return vol * m;
    };
    const auto dense = [&](const std::array<Vector3d, 4>& q) {
        const int K = 16;
        double sum = 0.;
        int n = 0;
        for (int a = 0; a <= K; ++a)
            for (int b = 0; a + b <= K; ++b)
                for (int c = 0; a + b + c <= K; ++c) {
                    const int d = K - a - b - c;
                    const Vector3d p = (a * q[0] + b * q[1] + c * q[2] + d * q[3]) / double(K);
                    sum += (d_all(p) - delta) / delta;
                    ++n;
                }
        return std::abs((q[1] - q[0]).dot((q[2] - q[0]).cross(q[3] - q[0]))) / 6. * sum / n;
    };

    SECTION("an upper bound, and a split never raises it")
    {
        // Cells outside the cube near an edge and a corner (d curved), and one spanning across
        // the cube's corner region.
        std::uniform_real_distribution<double> J(-0.15, 0.15);
        int checked = 0;
        for (int k = 0; k < 60; ++k) {
            const Vector3d c0 = k % 3 == 0   ? Vector3d(1.1, 0.2, 1.1)
                                : k % 3 == 1 ? Vector3d(1.1, 1.1, 1.1)
                                             : Vector3d(1.05, 0.0, 1.25);
            std::array<Vector3d, 4> q;
            for (auto& p : q) {
                p = c0 + Vector3d(J(rng), J(rng), J(rng));
                for (int j = 0; j < 3; ++j) p[j] = std::max(p[j], c0[j] > 1. ? 1.0 + 1e-3 : -1.);
            }
            const double vol = std::abs((q[1] - q[0]).dot((q[2] - q[0]).cross(q[3] - q[0]))) / 6.;
            if (vol < 1e-5) continue;
            int64_t best = -1;
            const double u = rule(q, -1, best);
            INFO("cell " << k << " rule " << u << " dense " << dense(q));
            CHECK(u >= dense(q) - 1e-9 * std::max(1., std::abs(u)));
            // split the longest edge at its midpoint into two children
            int ia = 0, ib = 1;
            for (int a = 0; a < 4; ++a)
                for (int b = a + 1; b < 4; ++b)
                    if ((q[a] - q[b]).norm() > (q[ia] - q[ib]).norm()) ia = a, ib = b;
            const Vector3d mid = 0.5 * (q[ia] + q[ib]);
            std::array<Vector3d, 4> c1 = q, c2 = q;
            c1[ib] = mid;
            c2[ia] = mid;
            int64_t b1 = -1, b2 = -1;
            const double kids = rule(c1, best, b1) + rule(c2, best, b2);
            INFO("children " << kids);
            CHECK(kids <= u + 1e-12 * std::max(1., std::abs(u)));
            ++checked;
        }
        CHECK(checked > 40);
    }

    SECTION("BandVolumeEnergy3D corner bound: value and derivatives")
    {
        auto env = std::make_shared<SampleEnvelope>();
        env->use_exact = true;
        {
            std::vector<Eigen::Vector3d> verts(size_t(V.rows()));
            for (int i = 0; i < V.rows(); ++i) verts[size_t(i)] = V.row(i).transpose();
            std::vector<Eigen::Vector3i> tv(size_t(F.rows()));
            for (int i = 0; i < F.rows(); ++i)
                tv[size_t(i)] = Eigen::Vector3i(F(i, 0), F(i, 1), F(i, 2));
            env->init(verts, tv, delta);
        }
        const auto pot = std::make_shared<const EuclideanOffsetPotential3D>(env, delta);
        const Vector3d ctr(1.06, 0.0, 1.07); // beyond the edge x = z = 1
        const double rad = 0.03, w = 0.7;
        std::vector<BandVolumeEnergy3D::Cell> cells;
        std::vector<std::vector<int64_t>> cand;
        for (const double sx : {-1., 1.})
            for (const double sy : {-1., 1.})
                for (const double sz : {-1., 1.}) {
                    if (sz > 0.) continue;
                    BandVolumeEnergy3D::Cell cl;
                    cl.q1 = ctr + sx * rad * Vector3d::UnitX();
                    cl.q2 = ctr + sy * rad * Vector3d::UnitY();
                    cl.q3 = ctr + sz * rad * Vector3d::UnitZ();
                    std::vector<int64_t> cc;
                    for (const Vector3d& p : {ctr, cl.q1, cl.q2, cl.q3})
                        cc.push_back(tris->nearest(p));
                    std::sort(cc.begin(), cc.end());
                    cc.erase(std::unique(cc.begin(), cc.end()), cc.end());
                    cells.push_back(cl);
                    cand.push_back(cc);
                }
        const auto cells_copy = cells;
        BandVolumeEnergy3D energy(pot, cells, ctr, w);
        energy.set_corner_bound(tris, cand, delta);
        const VectorXd xv = ctr + Vector3d(0.002, 0.001, -0.003);
        // value against the rule written out (the constructor may swap q2/q3; the corner mean does
        // not care)
        double want = 0.;
        for (size_t k = 0; k < cells_copy.size(); ++k) {
            const auto& cl = cells_copy[k];
            const std::array<Vector3d, 4> q = {Vector3d(xv), cl.q1, cl.q2, cl.q3};
            double m = std::numeric_limits<double>::infinity();
            for (const int64_t P : cand[k]) {
                double sum = 0.;
                for (const Vector3d& p : q) sum += (tris->distance(P, p) - delta) / delta;
                m = std::min(m, sum / 4.);
            }
            want += std::abs((q[1] - q[0]).dot((q[2] - q[0]).cross(q[3] - q[0]))) / 6. * m;
        }
        CHECK(energy.value(xv) == Catch::Approx(w * want).epsilon(1e-12));
        VectorXd g;
        energy.gradient(xv, g);
        MatrixXd H;
        energy.hessian(xv, H);
        const double h = 1e-7, gs = std::max(1e-9, g.norm()), hs = std::max(1e-9, H.norm());
        for (int k = 0; k < 3; ++k) {
            VectorXd xp = xv, xm = xv;
            xp[k] += h;
            xm[k] -= h;
            CHECK(std::abs((energy.value(xp) - energy.value(xm)) / (2. * h) - g[k]) <= 1e-5 * gs);
            VectorXd gp, gm;
            energy.gradient(xp, gp);
            energy.gradient(xm, gm);
            for (int j = 0; j < 3; ++j)
                CHECK(std::abs((gp[j] - gm[j]) / (2. * h) - H(j, k)) <= 1e-4 * hs);
        }
    }
}

TEST_CASE("cubed-amips-energy-3d-volume-weighted-flattening", "[offset][potential]")
{
    // A cell flattening at fixed edge lengths: vol AMIPS^3 grows like 1/height, never collapses
    // to 0 the way a floating-point determinant of a nearly flat cell can.
    const Vector3d q1(1., 0., 0.), q2(0.5, 0.9, 0.),
        q3(0.4, 0.3, 0.); // the face opposite x, in z = 0
    double prev = 0.;
    for (const double hgt : {1e-2, 1e-4, 1e-6, 1e-8, 1e-10}) {
        std::array<double, 12> c{};
        Vector3d a = q1, b = q2, d = q3;
        if (!wmtk::utils::orient3d(Vector3d(0.6, 0.4, hgt), a, b, d)) std::swap(b, d);
        for (int j = 0; j < 3; ++j) {
            c[size_t(3 + j)] = a[j];
            c[size_t(6 + j)] = b[j];
            c[size_t(9 + j)] = d[j];
        }
        CubedAMIPSEnergy3D e({c}, 1.);
        e.set_volume_weighted(true);
        const double v = e.value(VectorXd(Vector3d(0.6, 0.4, hgt)));
        INFO("height " << hgt << " value " << v);
        CHECK(std::isfinite(v));
        if (prev > 0.) CHECK(v > 50. * prev); // 1/height: x100 per step, allow slack
        prev = v;
    }
}

TEST_CASE("rest-amips-energy-3d-derivatives", "[offset][potential]")
{
    // The rest-shape AMIPS of a tet: gradient and Hessian against finite differences, and the
    // minimum of 3 at the rest shape (F = I), the same scale as the shared AMIPS against the
    // regular tet. The two cells carry different per-cell factors (Cell::weight, the rest
    // volume at a front vertex), so the derivatives are checked with the factor in them.
    std::vector<RestAMIPSEnergy3D::Cell> cells;
    {
        RestAMIPSEnergy3D::Cell c;
        c.q1 = Vector3d(1., 0.1, 0.);
        c.q2 = Vector3d(0.2, 1.1, 0.05);
        c.q3 = Vector3d(0.1, 0.3, 0.9);
        Eigen::Matrix3d R;
        R.col(0) = Vector3d(0.9, 0., 0.1);
        R.col(1) = Vector3d(0.1, 1., 0.);
        R.col(2) = Vector3d(0., 0.2, 1.);
        c.rest_inv = R.inverse();
        c.weight = 0.4;
        cells.push_back(c);
    }
    {
        RestAMIPSEnergy3D::Cell c;
        c.q1 = Vector3d(-0.9, 0.2, 0.1);
        c.q2 = Vector3d(-0.1, -0.8, 0.3);
        c.q3 = Vector3d(0.1, 0.1, -1.);
        Eigen::Matrix3d R;
        R.col(0) = Vector3d(-1., 0., 0.);
        R.col(1) = Vector3d(0., -1., 0.);
        R.col(2) = Vector3d(0., 0., -1.);
        c.rest_inv = R.inverse();
        c.weight = 2.5;
        cells.push_back(c);
    }
    RestAMIPSEnergy3D energy(cells, 1.3);

    const double h = 1e-6;
    for (const Vector3d& x :
         {Vector3d(0.05, -0.02, 0.03), Vector3d(-0.1, 0.1, 0.), Vector3d(0., 0., 0.)}) {
        VectorXd xv = x;
        VectorXd g;
        energy.gradient(xv, g);
        MatrixXd H;
        energy.hessian(xv, H);
        for (int k = 0; k < 3; ++k) {
            VectorXd xp = xv, xm = xv;
            xp[k] += h;
            xm[k] -= h;
            const double fd = (energy.value(xp) - energy.value(xm)) / (2. * h);
            INFO("k " << k << " fd " << fd << " analytic " << g[k]);
            CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1., std::abs(g[k])));
            VectorXd gp, gm;
            energy.gradient(xp, gp);
            energy.gradient(xm, gm);
            for (int j = 0; j < 3; ++j) {
                const double fdh = (gp[j] - gm[j]) / (2. * h);
                INFO("H(" << j << "," << k << ") fd " << fdh << " analytic " << H(j, k));
                CHECK(std::abs(fdh - H(j, k)) <= 1e-4 * std::max(1., std::abs(H(j, k))));
            }
        }
    }

    // At the rest shape every cell reads exactly 3 (one cell whose rest IS its current shape).
    RestAMIPSEnergy3D::Cell rest;
    rest.q1 = Vector3d(1., 0., 0.);
    rest.q2 = Vector3d(0., 1., 0.);
    rest.q3 = Vector3d(0., 0., 1.);
    rest.rest_inv = Eigen::Matrix3d::Identity();
    RestAMIPSEnergy3D at_rest({rest}, 1.0);
    VectorXd origin = Vector3d::Zero();
    CHECK(at_rest.value(origin) == Catch::Approx(3.));
    // The per-cell factor multiplies the cell's term, on top of the energy's own weight.
    RestAMIPSEnergy3D::Cell rest_v = rest;
    rest_v.weight = 0.25;
    RestAMIPSEnergy3D at_rest_v({rest_v}, 2.0);
    CHECK(at_rest_v.value(origin) == Catch::Approx(1.5));
    VectorXd g0;
    at_rest.gradient(origin, g0);
    CHECK(g0.norm() <= 1e-12);
    // Inverted is invalid, not merely bad.
    VectorXd bad = Vector3d(2., 2., 2.);
    CHECK(std::isnan(at_rest.value(bad)));
    CHECK(!at_rest.is_step_valid(origin, bad));

    // Cubed (the form every 3D smoother uses): each cell's term is pAMIPS^3, 27 at the rest
    // shape, with the chain-rule derivatives against finite differences.
    RestAMIPSEnergy3D at_rest3({rest_v}, 2.0, true);
    CHECK(at_rest3.value(origin) == Catch::Approx(2.0 * 0.25 * 27.));
    RestAMIPSEnergy3D cubed(cells, 1.3, true);
    for (const Vector3d& x :
         {Vector3d(0.05, -0.02, 0.03), Vector3d(-0.1, 0.1, 0.), Vector3d(0., 0., 0.)}) {
        VectorXd xv = x;
        VectorXd g;
        cubed.gradient(xv, g);
        MatrixXd H;
        cubed.hessian(xv, H);
        // value is the cube of the first-power energy cell by cell
        double expected = 0.;
        for (const RestAMIPSEnergy3D::Cell& c : cells) {
            RestAMIPSEnergy3D one({c}, 1.0);
            const double a = one.value(xv) / c.weight;
            expected += 1.3 * c.weight * a * a * a;
        }
        CHECK(cubed.value(xv) == Catch::Approx(expected).epsilon(1e-12));
        for (int k = 0; k < 3; ++k) {
            VectorXd xp = xv, xm = xv;
            xp[k] += h;
            xm[k] -= h;
            const double fd = (cubed.value(xp) - cubed.value(xm)) / (2. * h);
            INFO("cubed k " << k << " fd " << fd << " analytic " << g[k]);
            CHECK(std::abs(fd - g[k]) <= 1e-5 * std::max(1., std::abs(g[k])));
            VectorXd gp, gm;
            cubed.gradient(xp, gp);
            cubed.gradient(xm, gm);
            for (int j = 0; j < 3; ++j) {
                const double fdh = (gp[j] - gm[j]) / (2. * h);
                INFO("cubed H(" << j << "," << k << ") fd " << fdh << " analytic " << H(j, k));
                CHECK(std::abs(fdh - H(j, k)) <= 1e-4 * std::max(1., std::abs(H(j, k))));
            }
        }
    }
}


TEST_CASE("offset-energy-3d-lands-on-the-level-set", "[offset][potential]")
{
    // The end-to-end question, in 3D: does minimising w (Phi - c)^2 with the solver the smoother
    // uses put a vertex on the level set? Run without the mesh, so nothing here can be blamed on
    // inversion, the quality veto or the envelope.
    const double delta = 0.25;
    MatrixXd V(1, 3);
    V << 0., 0., 0.;
    const auto phi = std::make_shared<const SmoothOffsetPotential3D>(
        V,
        MatrixXi(0, 2),
        MatrixXi(0, 3),
        std::vector<int>{0},
        delta,
        DHAT_FACTOR);

    auto solver = wmtk::optimization::create_basic_solver();

    for (const double d0 : {0.5 * delta, 0.75 * delta, 1.0 * delta, 1.3 * delta, 1.8 * delta}) {
        auto energy = std::make_shared<OffsetEnergy3D>(phi, 1.0);
        VectorXd x(3);
        x << d0, 0., 0.;
        try {
            solver->minimize(*energy, x);
        } catch (const std::exception&) {
            // A failed line search is reported by throwing; the position reached is still the
            // best found, exactly as smooth_vertex_3d treats it.
        }
        const double d = x.norm();
        INFO("start " << d0 / delta << "x delta -> " << d / delta << "x delta");
        CHECK(d == Catch::Approx(delta).epsilon(0.02));
    }
}
