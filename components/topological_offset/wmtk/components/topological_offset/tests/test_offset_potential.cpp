#include <wmtk/components/topological_offset/OffsetPotential.hpp>

#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/solver.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Eigenvalues>

#include <cmath>
#include <set>
#include <utility>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::topological_offset;

/**
 * The calibration gate for the smooth offset potential.
 *
 * Nothing downstream of Phi means anything until these pass: the offset is DEFINED as the level
 * set Phi = c, so if Phi is not what we think it is, every distance the component reports is a
 * measurement of the wrong thing. The suite answers three separate questions.
 *
 *   1. Are the derivatives the derivatives? Central finite differences against grad Phi and
 *      hess Phi. These are ipc-toolkit's analytic expressions reached through our wrapper, so
 *      what is really under test is the wrapper -- the collision-set construction, the query
 *      row, and the block extraction.
 *
 *   2. Is the calibration self-consistent? Phi = c must recover exactly delta on a straight
 *      edge, since that is how c is defined.
 *
 *   3. HOW FAR IS THE SMOOTHED OFFSET FROM THE EUCLIDEAN ONE? This is the question the whole
 *      idea rests on, and the answer is deliberately not "zero": Phi sums the contributions of
 *      every feasible primitive, so a reentrant corner gets two barriers where the Euclidean
 *      distance gets one, and the level set bulges. The tests below pin the deviation on the
 *      four shapes where the exact Euclidean offset is known in closed form, so that a change
 *      to the potential, its parameters, or the upstream pin shows up as a number moving rather
 *      than as a mesh looking slightly different.
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
 * @brief Points inside the support, in the INTERIOR of a single primitive's feasible region.
 *
 * Both halves of that matter for a finite-difference test. Inside the support, or Phi is
 * identically zero and there is nothing to differentiate; away from a region boundary by more
 * than the FD step, or the two sides of the difference are evaluated against different active
 * sets and the comparison is meaningless. Sampling perpendicular to the middle of each segment
 * gives both: the projection is interior by construction, and the perpendicular offsets stay
 * within the support as long as they are under dhat.
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
 * The level set is what the offset IS, so every geometric comparison below goes through here.
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

/// A triangle soup as OffsetPotential<3> takes it. `E` is not optional and not a convenience:
/// ipc derives faces_to_edges from it and throws if an edge of a face is missing, and the OGC
/// feasible-region test for a VERTEX reads that vertex's edge neighbours -- so an incomplete
/// edge list would silently widen every Voronoi region.
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
/// set is the Euclidean offset of the POLYHEDRON -- which is what the test measures against.
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

/// Points inside the support and inside ONE primitive's feasible region, for finite differences.
/// Perpendicular to the interior of each triangle: the projection is interior by construction,
/// and the perpendicular offsets stay within the support as long as they are under dhat. Both
/// halves matter -- outside the support there is nothing to differentiate, and near a region
/// boundary the two sides of the difference see different active sets.
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
    // The calibration itself, restated as a test on a DIFFERENT straight edge from the one the
    // constructor calibrates on. c is defined as Phi at distance delta from a long straight
    // edge, so a flat stretch of any input must put the level set at exactly delta.
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

    // ... and the residual is a LENGTH that agrees with the true one to first order.
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
    // A convex closed curve. Every point outside is in exactly one primitive's feasible region
    // (the polyline is convex, so no two contributions overlap), which means the level set is
    // the Euclidean offset of the POLYLINE -- and the polyline is inscribed in the circle, so
    // the residual measured here is the polygonal discretisation, not the potential.
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
    // grown by delta with quarter-circle corners of radius delta, and both halves of that are
    // single-primitive regions -- a side's interior for the flats, a corner vertex for the
    // arcs -- so the smoothed offset should reproduce it closely.
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
    for (int i = 0; i <= 8; ++i) {
        const double a = 0.5 * M_PI * i / 8; // from +x to +y about the (h,h) corner
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
    // THE CASE WHERE THE TWO OFFSETS GENUINELY DIFFER, and the reason this suite reports
    // numbers rather than only asserting.
    //
    // Outside a REENTRANT corner both adjacent segments claim the point (it projects into the
    // interior of each), so Phi is the SUM of two barriers where the Euclidean distance is the
    // min of two distances. Phi is therefore larger than the single-pair value at the same
    // distance, and since Phi decreases outward the level set sits FURTHER OUT than delta. The
    // effect is largest on the corner's bisector, where the two contributions are equal, and
    // dies away along each arm as the second contribution leaves the support.
    //
    // This is the smoothing, not an error: it is what rounds the offset's own concave corner
    // instead of leaving the crease the Euclidean offset has there.
    const double delta = 0.1;
    const double half_angle = M_PI / 4.; // arms 45 degrees either side of +y: a right-angle notch

    // Two arms meeting at the origin and opening UPWARD, so a point on the +y bisector sits in
    // the notch between them and projects into the INTERIOR of both -- which is what makes the
    // corner reentrant as seen from that point, and what puts two barriers in the sum. (Arms
    // opening the other way give a convex corner, where only the corner vertex is feasible and
    // the level set is the exact circular arc -- that case is covered by the square above.)
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

    // Pushed OUTWARD, never inward. The bound is documentation of a measurement, not a
    // tolerance anyone tuned: it is what summing two nearly equal barriers does.
    CHECK(euclid_bisector > delta);
    CHECK(euclid_bisector <= 1.5 * delta);

    // Far along one arm, only that arm is within the support and the offset is exact again --
    // so the smoothing is LOCAL to the feature, which is the property that makes it usable.
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
        // which is the whole reason it is the default. Up to roundoff, RELATIVE to the matrix:
        // its scale is 2 w |grad Phi|^2 / c^2 (the energy is ((Phi - c)/c)^2 since 2026-08-25),
        // and the exact zero eigenvalue lands a few ulps of that on either side of zero. An
        // absolute 1e-12 failed at c ~ 0.17 for exactly that reason.
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
    // THE END-TO-END QUESTION THIS WHOLE DESIGN RESTS ON: does minimising w (Phi - c)^2 with
    // the solver the smoother actually uses put a vertex on the level set? Everything above
    // tests the field; this tests that the field is USABLE as an objective.
    //
    // Run without the mesh, so nothing here can be blamed on inversion, the quality veto or the
    // envelope: one free point, one energy, the same DenseNewton the smoother builds.
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
// 3D. The same three questions, plus one that only exists in 3D: does the broad phase seed all
// THREE candidate sets?
//
// In 2D, ipc's Candidates::vv_set() derives the vertex candidates from the endpoints of the edge
// candidates, so seeding the edge set alone is enough. The 3D builder reads vf_set, ve_set and
// vv_set independently and derives nothing. A missing set is silent and looks harmless: the
// pairs it would have contributed simply do not appear, Phi is SMALLER than it should be, and
// the level set has a hole exactly at the feature that primitive represents. The cube below is
// the cheapest thing that catches it -- its three probes (face interior, edge, corner) are
// claimed by exactly one FACE, one EDGE and one VERTEX respectively, so each of the three sets
// is separately load-bearing for one of the three CHECKs.
// =============================================================================================


TEST_CASE("offset-potential-3d-calibration", "[offset][potential]")
{
    // c is DEFINED as Phi at perpendicular distance delta from one large flat primitive, and the
    // barrier only ever sees a distance -- so the level a 3D run places its offset on must be the
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

    // ... and a flat stretch of a DIFFERENT input puts the level set at exactly delta.
    for (const Vector3d& o : {Vector3d(0., 0., 3.), Vector3d(1., -2., 3.), Vector3d(-3., 1., 3.)}) {
        const double r =
            level_set_radius<3>(phi3, o, Vector3d(0., 0., 1.), 0.05 * delta, 0.999 * phi3.dhat());
        CHECK(r == Catch::Approx(delta).epsilon(1e-9));
    }

    // The residual is a LENGTH, agreeing with the true one to first order at the level set.
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
    // THE BROAD-PHASE TEST (see the section note above). Outside a convex cube every point is
    // claimed by exactly ONE primitive, and which one depends only on where it is:
    //
    //   above a face interior -> that triangle    (needs vf_set)
    //   beside an edge        -> that edge        (needs ve_set)
    //   beyond a corner       -> that vertex      (needs vv_set)
    //
    // so the exact Euclidean offset of the cube -- flat panels, quarter-cylinders along the
    // edges, eighth-spheres at the corners -- is also the level set, to machine precision. A
    // candidate set that is never seeded shows up here as a CHECK that cannot even bracket the
    // level set, because Phi is identically zero along that probe.
    const double h = 1.0;
    const double delta = 0.1;
    const TriSoup s = cube(h);
    const SmoothOffsetPotential3D phi(s.V, s.E, s.F, {}, delta, DHAT_FACTOR);

    // FACES. Straight out of every triangle's centroid, which is interior by construction.
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

    // EDGES. Out of the middle of each of the 12 cube edges, along the diagonal of the two faces
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

    // CORNERS. Out along the body diagonal from each of the 8 corners: the exact offset is the
    // eighth-sphere of radius delta about the corner. This is the probe the 2D code gets for
    // free and 3D does not.
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
    // A convex closed surface. Every point outside is in exactly one primitive's feasible region,
    // so the level set is the Euclidean offset of the POLYHEDRON -- which is INSCRIBED in the
    // sphere, so measuring it against the sphere charges the potential for the triangulation.
    //
    // The 2D circle test can ignore that (a 256-gon's sagitta is 0.08% of delta); a UV sphere's
    // is not negligible, so the test refines instead of loosening. A face's worst point is its
    // centroid, at circumradius^2 / 2R inside the sphere, which is QUADRATIC in the edge length --
    // so quadrupling the resolution must cut the deviation by far more than the 4x asserted
    // below, and does (measured 3.83% -> 0.24% of delta). That convergence is the actual claim:
    // what is left is the mesh, not Phi.
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
    // A 1-DIMENSIONAL input in 3D: one segment, no triangles. Nothing derives its candidates
    // from anything, so this exercises the edge tree and the isolated-edge branch directly.
    //
    // The exact offset of a segment is a capsule: a cylinder of radius delta along its length,
    // capped by hemispheres. Both halves are single-primitive regions here -- ipc's edge
    // feasible-region test degenerates to "the projection is interior" when the edge has no
    // incident triangle, and the endpoints claim everything beyond.
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
    for (int i = 0; i <= 6; ++i) {
        const double a = 0.5 * M_PI * i / 6; // from +x round to +y about the (1,0,0) end
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
    // A 0-DIMENSIONAL input in 3D. Besides the geometry, this is what exercises the SENTINEL
    // segment: ipc's are_adjacencies_initialized() is false when the mesh has no edges at all,
    // and every accessor the feasible-region test calls then throws.
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
    // THE CASE WHERE THE TWO OFFSETS GENUINELY DIFFER, in 3D: a right-angle notch between two
    // quads meeting along the y axis. A point on the bisector projects into the INTERIOR of both,
    // so Phi is the SUM of two barriers where the Euclidean distance is the min of two distances,
    // and since Phi decreases outward the level set sits FURTHER OUT than delta.
    //
    // This is the smoothing, not an error: it is what rounds the offset's own concave crease
    // instead of leaving the kink the Euclidean offset has there. Far from the crease only one
    // quad is within the support and the offset is exact again -- the effect is LOCAL to the
    // feature, which is the property that makes it usable.
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
    // Pushed OUTWARD, never inward. The bound documents a measurement, not a tuned tolerance:
    // it is what summing two nearly equal barriers does.
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


TEST_CASE("offset-energy-3d-lands-on-the-level-set", "[offset][potential]")
{
    // THE END-TO-END QUESTION, in 3D: does minimising w (Phi - c)^2 with the solver the smoother
    // actually uses put a vertex on the level set? Run without the mesh, so nothing here can be
    // blamed on inversion, the quality veto or the envelope: one free point, one energy, the same
    // DenseNewton the smoother builds.
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
