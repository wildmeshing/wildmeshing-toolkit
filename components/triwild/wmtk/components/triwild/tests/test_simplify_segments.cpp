#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/SimplifySegments.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <set>
#include <vector>

using namespace wmtk;

namespace {

/// Build an envelope of thickness `eps` around (V, E).
SampleEnvelope make_envelope(const MatrixXd& V, const MatrixXi& E, double eps)
{
    std::vector<Vector2d> v(V.rows());
    for (int i = 0; i < V.rows(); ++i) {
        v[i] = Vector2d(V(i, 0), V(i, 1));
    }
    std::vector<Vector2i> e(E.rows());
    for (int i = 0; i < E.rows(); ++i) {
        e[i] = Vector2i(E(i, 0), E(i, 1));
    }
    SampleEnvelope env;
    env.init(v, e, eps);
    return env;
}

/// A closed polygon through the given points.
std::pair<MatrixXd, MatrixXi> closed_loop(const std::vector<Vector2d>& pts)
{
    const int n = static_cast<int>(pts.size());
    MatrixXd V(n, 2);
    MatrixXi E(n, 2);
    for (int i = 0; i < n; ++i) {
        V.row(i) = pts[i];
        E(i, 0) = i;
        E(i, 1) = (i + 1) % n;
    }
    return {V, E};
}

/// An open polyline through the given points.
std::pair<MatrixXd, MatrixXi> open_chain(const std::vector<Vector2d>& pts)
{
    const int n = static_cast<int>(pts.size());
    MatrixXd V(n, 2);
    MatrixXi E(n - 1, 2);
    for (int i = 0; i < n; ++i) {
        V.row(i) = pts[i];
    }
    for (int i = 0; i + 1 < n; ++i) {
        E(i, 0) = i;
        E(i, 1) = i + 1;
    }
    return {V, E};
}

std::vector<Vector2d> sample_circle(int n, double r = 1.0)
{
    std::vector<Vector2d> pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i) {
        const double t = 2.0 * M_PI * i / n;
        pts.emplace_back(r * std::cos(t), r * std::sin(t));
    }
    return pts;
}

/// Distance from p to the segment network (V, E).
double distance_to(const MatrixXd& V, const MatrixXi& E, const Vector2d& p)
{
    double best = std::numeric_limits<double>::max();
    for (int i = 0; i < E.rows(); ++i) {
        const Vector2d a = V.row(E(i, 0));
        const Vector2d b = V.row(E(i, 1));
        const Vector2d ab = b - a;
        const double l2 = ab.squaredNorm();
        double t = l2 > 0 ? (p - a).dot(ab) / l2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        best = std::min(best, (p - (a + t * ab)).norm());
    }
    return best;
}

/// No segment may be degenerate or duplicated.
void check_well_formed(const MatrixXd& V, const MatrixXi& E)
{
    std::set<std::pair<int, int>> seen;
    for (int i = 0; i < E.rows(); ++i) {
        const int a = E(i, 0), b = E(i, 1);
        REQUIRE(a != b);
        REQUIRE(a >= 0);
        REQUIRE(b >= 0);
        REQUIRE(a < V.rows());
        REQUIRE(b < V.rows());
        REQUIRE(seen.insert({std::min(a, b), std::max(a, b)}).second);
    }
}

} // namespace

TEST_CASE("simplify-segments-dense-circle", "[triwild_operation][simplify]")
{
    // A circle sampled far finer than the tolerance collapses hard, and every point of the
    // original stays within eps of what is left.
    auto [V, E] = closed_loop(sample_circle(1000));
    const MatrixXd V0 = V;
    const MatrixXi E0 = E;

    const double eps = 1e-2;
    const SampleEnvelope env = make_envelope(V, E, eps);

    const size_t removed = wmtk::utils::simplify_segments(V, E, env);

    CHECK(removed > 900);
    CHECK(E.rows() < 100);
    CHECK(E.rows() >= 3); // a closed curve cannot go below a triangle
    check_well_formed(V, E);

    for (int i = 0; i < V0.rows(); ++i) {
        CHECK(distance_to(V, E, Vector2d(V0(i, 0), V0(i, 1))) <= eps + 1e-12);
    }
}

TEST_CASE("simplify-segments-keeps-corners", "[triwild_operation][simplify]")
{
    // A square sampled densely along its sides: the sides collapse, the four corners must
    // not, because cutting one leaves the envelope.
    std::vector<Vector2d> pts;
    const int per_side = 50;
    const std::array<Vector2d, 4> corner = {
        Vector2d(0, 0),
        Vector2d(1, 0),
        Vector2d(1, 1),
        Vector2d(0, 1)};
    for (int c = 0; c < 4; ++c) {
        const Vector2d& p = corner[c];
        const Vector2d& q = corner[(c + 1) % 4];
        for (int i = 0; i < per_side; ++i) {
            pts.push_back(p + (q - p) * (double(i) / per_side));
        }
    }
    auto [V, E] = closed_loop(pts);

    const double eps = 1e-3; // far smaller than the corner deviation
    const SampleEnvelope env = make_envelope(V, E, eps);
    wmtk::utils::simplify_segments(V, E, env);

    check_well_formed(V, E);
    // The four corners survive, at their exact positions.
    for (const Vector2d& c : corner) {
        bool found = false;
        for (int i = 0; i < V.rows(); ++i) {
            if (V(i, 0) == c[0] && V(i, 1) == c[1]) {
                found = true;
                break;
            }
        }
        CHECK(found);
    }
    // and the sides really did collapse
    CHECK(E.rows() < 40);
}

TEST_CASE("simplify-segments-keeps-open-endpoints", "[triwild_operation][simplify]")
{
    // A straight open chain collapses to a single segment between its two endpoints, which
    // are frozen and must keep their exact positions.
    std::vector<Vector2d> pts;
    for (int i = 0; i <= 100; ++i) {
        pts.emplace_back(double(i) / 100.0, 0.0);
    }
    auto [V, E] = open_chain(pts);

    const SampleEnvelope env = make_envelope(V, E, 1e-3);
    wmtk::utils::simplify_segments(V, E, env);

    check_well_formed(V, E);
    CHECK(E.rows() == 1);
    const Vector2d p0 = V.row(E(0, 0));
    const Vector2d p1 = V.row(E(0, 1));
    CHECK(std::min(p0[0], p1[0]) == 0.0);
    CHECK(std::max(p0[0], p1[0]) == 1.0);
}

TEST_CASE("simplify-segments-keeps-junction", "[triwild_operation][simplify]")
{
    // A Y: three straight arms meeting at the origin. The arms collapse to one segment
    // each; the junction is valence 3, so it is frozen and keeps its exact position.
    const int per_arm = 40;
    const std::array<Vector2d, 3> dir = {
        Vector2d(1, 0),
        Vector2d(-0.5, 0.8660254),
        Vector2d(-0.5, -0.8660254)};

    std::vector<Vector2d> pts = {Vector2d(0, 0)};
    std::vector<Vector2i> segs;
    for (int a = 0; a < 3; ++a) {
        int prev = 0;
        for (int i = 1; i <= per_arm; ++i) {
            pts.push_back(dir[a] * (double(i) / per_arm));
            const int cur = static_cast<int>(pts.size()) - 1;
            segs.emplace_back(prev, cur);
            prev = cur;
        }
    }
    MatrixXd V(pts.size(), 2);
    for (size_t i = 0; i < pts.size(); ++i) {
        V.row(i) = pts[i];
    }
    MatrixXi E(segs.size(), 2);
    for (size_t i = 0; i < segs.size(); ++i) {
        E.row(i) = segs[i];
    }

    const SampleEnvelope env = make_envelope(V, E, 1e-3);
    wmtk::utils::simplify_segments(V, E, env);

    check_well_formed(V, E);
    CHECK(E.rows() == 3); // one segment per arm

    // The junction survived at the origin, still with valence 3.
    int junction = -1;
    for (int i = 0; i < V.rows(); ++i) {
        if (V(i, 0) == 0.0 && V(i, 1) == 0.0) {
            junction = i;
        }
    }
    REQUIRE(junction != -1);
    int valence = 0;
    for (int i = 0; i < E.rows(); ++i) {
        valence += (E(i, 0) == junction) + (E(i, 1) == junction);
    }
    CHECK(valence == 3);
}

TEST_CASE("simplify-segments-keeps-free-points", "[triwild_operation][simplify]")
{
    // Vertices with no incident segment are free points the arrangement still triangulates;
    // simplification must not drop them.
    auto [V, E] = closed_loop(sample_circle(200, 1.0));
    const int n = static_cast<int>(V.rows());
    V.conservativeResize(n + 2, 2);
    V.row(n) = Vector2d(0.0, 0.0);
    V.row(n + 1) = Vector2d(0.3, 0.2);

    const SampleEnvelope env = make_envelope(V, E, 1e-2);
    wmtk::utils::simplify_segments(V, E, env);

    check_well_formed(V, E);
    bool has_origin = false, has_other = false;
    for (int i = 0; i < V.rows(); ++i) {
        has_origin |= (V(i, 0) == 0.0 && V(i, 1) == 0.0);
        has_other |= (V(i, 0) == 0.3 && V(i, 1) == 0.2);
    }
    CHECK(has_origin);
    CHECK(has_other);
}

TEST_CASE("simplify-segments-deterministic", "[triwild_operation][simplify]")
{
    // Equal-length segments are everywhere in a uniformly sampled curve, so the queue order
    // has to be a total order. Two runs must agree exactly.
    auto [V1, E1] = closed_loop(sample_circle(512));
    MatrixXd V2 = V1;
    MatrixXi E2 = E1;

    const SampleEnvelope env1 = make_envelope(V1, E1, 5e-3);
    const SampleEnvelope env2 = make_envelope(V2, E2, 5e-3);
    const size_t r1 = wmtk::utils::simplify_segments(V1, E1, env1);
    const size_t r2 = wmtk::utils::simplify_segments(V2, E2, env2);

    CHECK(r1 == r2);
    REQUIRE(V1.rows() == V2.rows());
    REQUIRE(E1.rows() == E2.rows());
    CHECK(V1 == V2);
    CHECK(E1 == E2);
}

TEST_CASE("simplify-segments-already-coarse", "[triwild_operation][simplify]")
{
    // Nothing to do: a triangle is already minimal, and the input must come back untouched.
    auto [V, E] = closed_loop({Vector2d(0, 0), Vector2d(1, 0), Vector2d(0, 1)});
    const MatrixXd V0 = V;
    const MatrixXi E0 = E;

    const SampleEnvelope env = make_envelope(V, E, 1e-3);
    const size_t removed = wmtk::utils::simplify_segments(V, E, env);

    CHECK(removed == 0);
    CHECK(V == V0);
    CHECK(E == E0);
}
