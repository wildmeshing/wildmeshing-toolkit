#include <wmtk/components/topological_offset/OffsetPotential.hpp>

#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/solver.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Eigenvalues>

#include <cmath>
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
double level_set_radius(
    const OffsetPotential& phi,
    const Vector2d& origin,
    const Vector2d& dir,
    const double lo_in,
    const double hi_in)
{
    const Vector2d d = dir.normalized();
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
    const OffsetPotential phi(p.V, p.E, {}, delta, DHAT_FACTOR);

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
    const OffsetPotential phi(p.V, p.E, {}, delta, DHAT_FACTOR);

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
    const OffsetPotential phi(p.V, p.E, {}, delta, DHAT_FACTOR);

    // The closed form the calibration must agree with: one active Vertex2-Edge2P1 pair, so
    // Phi is exactly the normalized clamped-log barrier at the perpendicular distance.
    const double t = delta / phi.dhat();
    const double c_expected = -(t - 1.) * (t - 1.) * std::log(t);
    CHECK(phi.target_level() == Catch::Approx(c_expected).epsilon(1e-12));

    for (const double x : {-4., -1., 0., 2., 5.}) {
        const double r = level_set_radius(
            phi, Vector2d(x, 3.), Vector2d(0., 1.), 0.05 * delta, 0.999 * phi.dhat());
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
    const OffsetPotential phi(V, MatrixXi(0, 2), {0}, delta, DHAT_FACTOR);

    const Vector2d o(0.4, -0.7);
    double max_err = 0.;
    for (int i = 0; i < 32; ++i) {
        const double a = 2. * M_PI * i / 32;
        const double r = level_set_radius(
            phi, o, Vector2d(std::cos(a), std::sin(a)), 0.05 * delta, 0.999 * phi.dhat());
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
    const OffsetPotential phi(
        circle_polyline(R, 256).V, circle_polyline(R, 256).E, {}, delta, DHAT_FACTOR);

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
        "circle R=" << R << " delta=" << delta << ": max |offset - delta| = " << max_err
                    << " (" << 100. * max_err / delta << "% of delta), mean "
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
    const Polyline p = closed_polyline(
        {Vector2d(-h, -h), Vector2d(h, -h), Vector2d(h, h), Vector2d(-h, h)});
    const OffsetPotential phi(p.V, p.E, {}, delta, DHAT_FACTOR);

    // Along the flats: the level set must sit at exactly h + delta.
    double flat_err = 0.;
    for (const double x : {-0.7, -0.3, 0.0, 0.4, 0.8}) {
        const double y = level_set_radius(
            phi, Vector2d(x, h), Vector2d(0., 1.), 0.05 * delta, 0.999 * phi.dhat());
        flat_err = std::max(flat_err, std::abs(y - delta));
    }
    INFO("square flat side: max |offset - delta| = " << flat_err << " (" << 100. * flat_err / delta
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
    INFO("square convex corner: max |offset - delta| = "
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
    const OffsetPotential phi(p.V, p.E, {}, delta, DHAT_FACTOR);

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
    const double d_far = level_set_radius(
        phi, 1.5 * dir, nrm, 0.05 * delta, 0.999 * phi.dhat());
    INFO("reentrant wedge, far along an arm: " << d_far << " vs delta " << delta);
    CHECK(d_far == Catch::Approx(delta).epsilon(0.01));
}


TEST_CASE("offset-potential-support", "[offset][potential]")
{
    const double delta = 0.1;
    const Polyline p = open_polyline({Vector2d(-5., 0.), Vector2d(5., 0.)});
    const OffsetPotential phi(p.V, p.E, {}, delta, DHAT_FACTOR);

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
    CHECK_THROWS(OffsetPotential(p.V, p.E, {}, delta, 1.0));
}


TEST_CASE("offset-energy-derivatives", "[offset][potential]")
{
    // The smoothing term itself: w (Phi - c)^2, and its gradient, against finite differences.
    const Polyline p = open_polyline(
        {Vector2d(-1., 0.), Vector2d(0., 0.), Vector2d(0.3, 0.4), Vector2d(0.9, 0.1)});
    const double delta = 0.1;
    const auto phi = std::make_shared<const OffsetPotential>(p.V, p.E, std::vector<int>{}, delta, DHAT_FACTOR);

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
        // which is the whole reason it is the default.
        MatrixXd H;
        energy.hessian(xv, H);
        const Eigen::SelfAdjointEigenSolver<MatrixXd> es(H);
        CHECK(es.eigenvalues().minCoeff() >= -1e-12);

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
    const auto phi =
        std::make_shared<const OffsetPotential>(V, MatrixXi(0, 2), std::vector<int>{0}, delta, DHAT_FACTOR);

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
