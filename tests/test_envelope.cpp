// Containment queries that only the exact envelope can answer.
//
// Both of SampleEnvelope's backends describe the SAME envelope: the eps-neighbourhood of the
// input. They are handed the same eps and neither is meant to be wider or narrower than the
// other. What differs is how much of that eps they manage to use.
//
// The sampled one places points along the query and asks the BVH whether each is within some
// radius of the input. It cannot see what happens between two samples, so to guarantee the
// whole query is within eps it has to shrink that radius by the sampling's covering radius --
// eps/sqrt(3) for a triangle, eps/2 for a segment (see SampleEnvelope::eps2 and eps2_edge).
// That shrink is an implementation detail of staying conservative, not a smaller envelope: it
// costs some of the eps budget, so the sampled test rejects geometry that is genuinely inside.
//
// The exact one decides coverage by the union of the input's eps-neighbourhoods directly, so
// it spends almost none of the budget on conservatism -- fast-envelope inscribes a hexahedron
// (3D) or rectangle (2D) whose corners reach exactly eps.
//
// Until fast-envelope gained envelopes built from edges, only the triangle-mesh envelope had
// an exact backend; the segment and 2D queries threw. These cases pin what the exact path now
// answers, and in particular the two places where the backends legitimately disagree: the band
// the sampled test gives up to its own conservatism, and a gap narrower than the sample
// spacing, where the sampled test is simply wrong.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/EnvelopeBudget.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/threading/task_group.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <thread>

using namespace wmtk;

namespace {

/// A single segment of the x axis, from the origin to (length, 0, 0).
void init_axis_3d(SampleEnvelope& env, double length, double eps)
{
    const std::vector<Eigen::Vector3d> V = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(length, 0, 0)};
    const std::vector<Eigen::Vector2i> E = {Eigen::Vector2i(0, 1)};
    env.init(V, E, eps);
}

/// The same in 2D.
void init_axis_2d(SampleEnvelope& env, double length, double eps)
{
    const std::vector<Eigen::Vector2d> V = {Eigen::Vector2d(0, 0), Eigen::Vector2d(length, 0)};
    const std::vector<Eigen::Vector2i> E = {Eigen::Vector2i(0, 1)};
    env.init(V, E, eps);
}

} // namespace

TEST_CASE("exact and sampled 3D segment envelopes agree away from the boundary", "[envelope]")
{
    // The two backends answer the same question in the interior and the far exterior. They are
    // allowed to differ only near the boundary, which the next case covers, so agreement here
    // is what says the exact path is wired to the right geometry at all -- a mis-dispatched
    // exact query would answer "outside" for everything, including the queries well inside.
    const double eps = 0.1;
    SampleEnvelope sampled(false);
    SampleEnvelope exact(true);
    init_axis_3d(sampled, 4.0, eps);
    init_axis_3d(exact, 4.0, eps);

    SECTION("well inside")
    {
        const std::array<Eigen::Vector3d, 2> e = {
            {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(3, 0.01, 0)}};
        CHECK_FALSE(sampled.is_outside(e));
        CHECK_FALSE(exact.is_outside(e));
    }
    SECTION("well outside, sideways")
    {
        const std::array<Eigen::Vector3d, 2> e = {
            {Eigen::Vector3d(1, 0.5, 0), Eigen::Vector3d(3, 0.5, 0)}};
        CHECK(sampled.is_outside(e));
        CHECK(exact.is_outside(e));
    }
    SECTION("well outside, past the end")
    {
        const std::array<Eigen::Vector3d, 2> e = {
            {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(6, 0, 0)}};
        CHECK(sampled.is_outside(e));
        CHECK(exact.is_outside(e));
    }
    SECTION("a point query is unaffected by which backend answers it")
    {
        CHECK_FALSE(sampled.is_outside(Eigen::Vector3d(2, 0, 0)));
        CHECK_FALSE(exact.is_outside(Eigen::Vector3d(2, 0, 0)));
        CHECK(sampled.is_outside(Eigen::Vector3d(2, 0.5, 0)));
        CHECK(exact.is_outside(Eigen::Vector3d(2, 0.5, 0)));
    }
}

TEST_CASE("the sampled segment envelope is the stricter of the two", "[envelope]")
{
    // Same envelope, different amounts of it recovered. The sampled test accepts a segment
    // only when every sample is within eps/2, because the spacing guarantees no more than
    // another eps/2 between samples -- so above eps/2 it starts rejecting geometry that is
    // inside the eps envelope it is supposed to represent. That is safe, and it is the cost of
    // sampling rather than a smaller envelope; recovering it is the point of the exact path.
    //
    // The offset below has to sit in the band the sampled test gives up. Its upper end is not
    // eps: fast-envelope inscribes a hexahedron of half-width eps/sqrt(3) whose corners reach
    // eps, so eps/sqrt(3) is the offset guaranteed inside whatever orientation the frame comes
    // out with. Using the inscribed sphere keeps this from depending on how seg_cube happens
    // to pick its two cross-section axes.
    //
    //     eps/2 = 0.0500  <  0.95 * eps/sqrt(3) = 0.0548  <=  eps/sqrt(3) = 0.0577
    const double eps = 0.1;
    const double inscribed = eps / std::sqrt(3.0);
    const double offset = 0.95 * inscribed;
    REQUIRE(offset > eps / 2); // sampled must reject
    REQUIRE(offset <= inscribed); // exact must accept

    SampleEnvelope sampled(false);
    SampleEnvelope exact(true);
    init_axis_3d(sampled, 4.0, eps);
    init_axis_3d(exact, 4.0, eps);

    const std::array<Eigen::Vector3d, 2> e = {
        {Eigen::Vector3d(1, offset, 0), Eigen::Vector3d(3, offset, 0)}};
    CHECK(sampled.is_outside(e));
    CHECK_FALSE(exact.is_outside(e));
}

TEST_CASE("the exact 2D envelope answers point and segment queries", "[envelope]")
{
    const double eps = 0.1;
    SampleEnvelope sampled(false);
    SampleEnvelope exact(true);
    init_axis_2d(sampled, 4.0, eps);
    init_axis_2d(exact, 4.0, eps);

    SECTION("points")
    {
        // is_outside(Vector2d) cannot reach the exact backend by lifting to z = 0 the way the
        // sampled one does -- the 2D envelope is a separate object -- so this is the case that
        // catches that dispatch being wrong.
        CHECK_FALSE(sampled.is_outside(Eigen::Vector2d(2, 0)));
        CHECK_FALSE(exact.is_outside(Eigen::Vector2d(2, 0)));
        CHECK(sampled.is_outside(Eigen::Vector2d(2, 0.5)));
        CHECK(exact.is_outside(Eigen::Vector2d(2, 0.5)));
    }
    SECTION("segments")
    {
        const std::array<Eigen::Vector2d, 2> inside = {
            {Eigen::Vector2d(1, 0), Eigen::Vector2d(3, 0)}};
        CHECK_FALSE(sampled.is_outside(inside));
        CHECK_FALSE(exact.is_outside(inside));

        const std::array<Eigen::Vector2d, 2> outside = {
            {Eigen::Vector2d(1, 0.5), Eigen::Vector2d(3, 0.5)}};
        CHECK(sampled.is_outside(outside));
        CHECK(exact.is_outside(outside));
    }
    SECTION("the 2D rectangle is inscribed in the eps-capsule, at eps/sqrt(2)")
    {
        const double inscribed = eps / std::sqrt(2.0);
        const std::array<Eigen::Vector2d, 2> e = {
            {Eigen::Vector2d(1, 0.9 * inscribed), Eigen::Vector2d(3, 0.9 * inscribed)}};
        CHECK_FALSE(exact.is_outside(e));

        const std::array<Eigen::Vector2d, 2> just_out = {
            {Eigen::Vector2d(1, 1.1 * inscribed), Eigen::Vector2d(3, 1.1 * inscribed)}};
        CHECK(exact.is_outside(just_out));
    }
}

TEST_CASE("only the exact envelope sees a gap between two input curves", "[envelope]")
{
    // Two collinear segments with a gap between them. A query spanning the gap leaves the
    // input and comes back, so it is genuinely outside the envelope -- but the sampled test
    // only looks at its samples, and when the gap is narrower than the sample spacing every
    // sample can land on covered ground. This is the failure the exact envelope exists to
    // prevent, and the reason a bridging segment must be rejected rather than merely "usually"
    // rejected: the sampled answer depends on where the samples happen to fall.
    const double eps = 0.1;
    const double gap = 0.3; // wide enough that no eps-neighbourhood spans it
    const std::vector<Eigen::Vector2d> V = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(1, 0),
        Eigen::Vector2d(1 + gap, 0),
        Eigen::Vector2d(2 + gap, 0)};
    const std::vector<Eigen::Vector2i> E = {Eigen::Vector2i(0, 1), Eigen::Vector2i(2, 3)};

    SampleEnvelope exact(true);
    exact.init(V, E, eps);

    // Inside one component: fine.
    CHECK_FALSE(exact.is_outside(
        std::array<Eigen::Vector2d, 2>{{Eigen::Vector2d(0.2, 0), Eigen::Vector2d(0.8, 0)}}));
    // Spanning the gap: outside, even though both endpoints are inside.
    CHECK(exact.is_outside(
        std::array<Eigen::Vector2d, 2>{{Eigen::Vector2d(0.5, 0), Eigen::Vector2d(1.8 + gap, 0)}}));
    // Both endpoints inside is not sufficient even for a short query.
    CHECK(exact.is_outside(
        std::array<Eigen::Vector2d, 2>{{Eigen::Vector2d(1, 0), Eigen::Vector2d(1 + gap, 0)}}));
}

TEST_CASE("an exact query against the wrong kind of envelope throws", "[envelope]")
{
    // The dangerous failure mode is silence: a FastEnvelope that no init ever filled in has no
    // prisms, so every query against it answers "outside" and the caller sees an optimization
    // where every operation is vetoed rather than an error. These are the combinations that
    // would otherwise land there.
    const double eps = 0.1;

    SECTION("triangle query against a 2D envelope")
    {
        SampleEnvelope exact(true);
        init_axis_2d(exact, 4.0, eps);
        const std::array<Eigen::Vector3d, 3> tri = {
            {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0)}};
        CHECK_THROWS(exact.is_outside(tri));
    }
    SECTION("triangle query against a 3D edge envelope")
    {
        SampleEnvelope exact(true);
        init_axis_3d(exact, 4.0, eps);
        const std::array<Eigen::Vector3d, 3> tri = {
            {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0)}};
        CHECK_THROWS(exact.is_outside(tri));
    }
    SECTION("3D segment query against a 2D envelope")
    {
        SampleEnvelope exact(true);
        init_axis_2d(exact, 4.0, eps);
        const std::array<Eigen::Vector3d, 2> e = {
            {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(3, 0, 0)}};
        CHECK_THROWS(exact.is_outside(e));
    }
    SECTION("an exact envelope needs a positive epsilon")
    {
        // Zero eps is legitimate for a nearest_point-only envelope (triwild's Delaunay
        // seeding does it), but there is no zero-width exact envelope to build.
        SampleEnvelope exact(true);
        CHECK_THROWS(init_axis_2d(exact, 4.0, 0.0));

        SampleEnvelope sampled(false);
        CHECK_NOTHROW(init_axis_2d(sampled, 4.0, 0.0));
    }
    SECTION("the sampled backend accepts every combination, as before")
    {
        SampleEnvelope sampled(false);
        init_axis_2d(sampled, 4.0, eps);
        const std::array<Eigen::Vector3d, 2> e = {
            {Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(3, 0, 0)}};
        CHECK_NOTHROW(sampled.is_outside(e));
    }
}

// The deviation budget: which eps the optimizer's envelope is allowed to spend.
//
// Every app promises the output stays within eps of the ORIGINAL input, and two stages move
// geometry towards that limit. These pin the arithmetic that shares the budget between them,
// and in particular the two ways of getting it wrong that this function exists to prevent:
// charging for a simplification that never ran, and letting the remainder go non-positive.
TEST_CASE("envelope_budget", "[envelope]")
{
    using wmtk::utils::optimization_envelope_eps;
    const double eps = 1e-3;
    const double simplify_eps = 4e-4;

    SECTION("envelope around the input gets the whole budget")
    {
        // The envelope measures against the thing the promise is about, so the two stages do
        // not add up and there is nothing to share -- whether or not a simplification ran.
        CHECK(optimization_envelope_eps(eps, simplify_eps, false, true) == eps);
        CHECK(optimization_envelope_eps(eps, simplify_eps, false, false) == eps);
    }

    SECTION("envelope around the simplified geometry gets the remainder")
    {
        CHECK(
            optimization_envelope_eps(eps, simplify_eps, true, true) ==
            Catch::Approx(eps - simplify_eps));
    }

    SECTION("nothing is charged when no simplification ran")
    {
        // The regression this function was extracted for. The "simplified" geometry IS the
        // input, so it already sits at the centre of the full envelope; narrowing would
        // confine the optimizer to a fraction of the tolerance its result is judged against,
        // and buy nothing.
        CHECK(optimization_envelope_eps(eps, simplify_eps, true, false) == eps);
    }

    SECTION("a remainder that would be non-positive falls back to the full eps")
    {
        // Never returns <= 0. SampleEnvelope::init SQUARES its argument, so a negative eps
        // would come back as a positive eps2 -- a silently smaller envelope rather than an
        // error -- and a zero one vetoes every operation.
        CHECK(optimization_envelope_eps(eps, eps, true, true) == eps);
        CHECK(optimization_envelope_eps(eps, 1.5 * eps, true, true) == eps);
        CHECK(optimization_envelope_eps(eps, 0.999999 * eps, true, true) > 0);
    }
}

TEST_CASE("a large sampled triangle query answers the same on several threads", "[envelope]")
{
    // Every sample is decided exactly -- within eps2 of some input facet or not -- so however
    // the samples are split among threads, the triangle is outside exactly when one sample is.
    // The query triangles are ~10 eps across, thousands of samples each, so with more than one
    // thread allowed they take the split path.
    const double eps = 0.1;
    const int n = 20; // a 20x20 grid of the square [0,10]^2 at z = 0
    std::vector<Eigen::Vector3d> V;
    std::vector<Eigen::Vector3i> F;
    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i) V.emplace_back(10.0 * i / n, 10.0 * j / n, 0);
    }
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const int a = j * (n + 1) + i;
            F.emplace_back(a, a + 1, a + n + 2);
            F.emplace_back(a, a + n + 2, a + n + 1);
        }
    }
    SampleEnvelope serial(false);
    SampleEnvelope split(false);
    serial.init(V, F, eps);
    split.init(V, F, eps);
    split.set_max_threads(8);

    // Idle workers for the split queries to take: tasks that overlap start one worker each.
    {
        threading::task_group tg;
        for (int i = 0; i < 8; ++i) {
            tg.run([]() { std::this_thread::sleep_for(std::chrono::milliseconds(2)); });
        }
        tg.wait();
    }

    using T = std::array<Eigen::Vector3d, 3>;
    // In the plane, inside the square: inside. One corner lifted well clear of eps, or pushed
    // out past the square's edge: outside.
    const T flat = {{Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(9, 1, 0), Eigen::Vector3d(1, 9, 0)}};
    const T lifted = {
        {Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(9, 1, 0), Eigen::Vector3d(1, 9, 1)}};
    const T past_edge = {
        {Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(10.5, 1, 0), Eigen::Vector3d(1, 9, 0)}};
    CHECK_FALSE(split.is_outside(flat));
    CHECK(split.is_outside(lifted));
    CHECK(split.is_outside(past_edge));

    // Random large triangles near the square, at heights around the acceptance radius, so
    // that both answers come up and many queries are decided by a single late sample.
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> xy(-0.3, 10.3);
    std::uniform_real_distribution<double> z(-0.06, 0.06);
    int outside = 0;
    const int trials = 200;
    for (int k = 0; k < trials; ++k) {
        T t;
        for (auto& p : t) p = Eigen::Vector3d(xy(rng), xy(rng), z(rng));
        const bool expected = serial.is_outside(t);
        CHECK(split.is_outside(t) == expected);
        outside += expected ? 1 : 0;
    }
    // Neither answer may be vacuous.
    CHECK(outside > 0);
    CHECK(outside < trials);
}

// The per-primitive queries: primitives_in_box, primitive_distance and
// primitive_triangle_distance, the distance to ONE input primitive rather than to their union.
// Closed forms in every closest-feature region of a triangle and of a segment, central
// differences for the derivatives, the box query against brute force, and the min over a box's
// primitives against the BVH's own nearest distance.

namespace {

/// The triangle of the closed-form and derivative cases below, deliberately not axis-aligned.
std::vector<Eigen::Vector3d> skew_triangle_vertices()
{
    return {
        Eigen::Vector3d(0.1, 0.2, 0.3),
        Eigen::Vector3d(1.3, -0.1, 0.5),
        Eigen::Vector3d(0.4, 1.1, -0.2)};
}

/// Expected Hessian of the distance at a point p with foot `foot`, on a feature of dimension
/// `dim` with unit direction e (dim 1 only): 0, (I - u u^T - e e^T)/d, (I - u u^T)/d.
Eigen::Matrix3d expected_hessian(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& foot,
    const int dim,
    const Eigen::Vector3d& e)
{
    if (dim == 2) return Eigen::Matrix3d::Zero();
    const double d = (p - foot).norm();
    const Eigen::Vector3d u = (p - foot) / d;
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity() - u * u.transpose();
    if (dim == 1) H -= e * e.transpose();
    return H / d;
}

void check_closed_form(
    const SampleEnvelope& env,
    const int64_t id,
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& foot,
    const int dim,
    const Eigen::Vector3d& e = Eigen::Vector3d::Zero())
{
    Eigen::Vector3d g;
    Eigen::Matrix3d H;
    const double d = env.primitive_distance(id, p, &g, &H);
    CHECK(d == Catch::Approx((p - foot).norm()).margin(1e-14));
    CHECK((g - (p - foot).normalized()).norm() <= 1e-14);
    CHECK((H - expected_hessian(p, foot, dim, e)).norm() <= 1e-13);
}

/// Central differences of primitive_distance's value against its gradient, and of its gradient
/// against its Hessian, at a point inside one closest-feature region of primitive `id`.
void check_derivatives(const SampleEnvelope& env, const int64_t id, const Eigen::Vector3d& p)
{
    const double h = 1e-6;
    Eigen::Vector3d g;
    Eigen::Matrix3d H;
    env.primitive_distance(id, p, &g, &H);
    for (int j = 0; j < 3; ++j) {
        Eigen::Vector3d pp = p, pm = p;
        pp[j] += h;
        pm[j] -= h;
        Eigen::Vector3d gp, gm;
        const double dp = env.primitive_distance(id, pp, &gp);
        const double dm = env.primitive_distance(id, pm, &gm);
        CHECK(std::abs((dp - dm) / (2 * h) - g[j]) <= 1e-6);
        for (int i = 0; i < 3; ++i) CHECK(std::abs((gp[i] - gm[i]) / (2 * h) - H(i, j)) <= 1e-5);
    }
}

/// trace(H) d: 0 for a triangle's interior, 1 for an edge or a segment's interior, 2 for a
/// vertex, an endpoint or a point -- the rank of the Hessian, so which region p is in.
int region_rank(const SampleEnvelope& env, const int64_t id, const Eigen::Vector3d& p)
{
    Eigen::Matrix3d H;
    const double d = env.primitive_distance(id, p, nullptr, &H);
    return int(std::lround(H.trace() * d));
}

/// n random triangles of size ~0.2 in the unit cube.
void random_triangles(
    std::mt19937& rng,
    const int n,
    std::vector<Eigen::Vector3d>& V,
    std::vector<Eigen::Vector3i>& F)
{
    std::uniform_real_distribution<double> U(0, 1), D(-0.1, 0.1);
    for (int f = 0; f < n; ++f) {
        const Eigen::Vector3d c(U(rng), U(rng), U(rng));
        for (int k = 0; k < 3; ++k) V.push_back(c + Eigen::Vector3d(D(rng), D(rng), D(rng)));
        F.emplace_back(3 * f, 3 * f + 1, 3 * f + 2);
    }
}

/// n random segments of length ~0.2 in the unit cube, then m isolated points as (i, i) entries.
void random_segments_and_points(
    std::mt19937& rng,
    const int n,
    const int m,
    std::vector<Eigen::Vector3d>& V,
    std::vector<Eigen::Vector2i>& E)
{
    std::uniform_real_distribution<double> U(0, 1), D(-0.1, 0.1);
    for (int s = 0; s < n; ++s) {
        const Eigen::Vector3d c(U(rng), U(rng), U(rng));
        V.push_back(c + Eigen::Vector3d(D(rng), D(rng), D(rng)));
        V.push_back(c + Eigen::Vector3d(D(rng), D(rng), D(rng)));
        E.emplace_back(int(V.size()) - 2, int(V.size()) - 1);
    }
    for (int k = 0; k < m; ++k) {
        V.emplace_back(U(rng), U(rng), U(rng));
        E.emplace_back(int(V.size()) - 1, int(V.size()) - 1);
    }
}

/// Whether the closed boxes [lo0, hi0] and [lo1, hi1] meet.
bool boxes_meet(
    const Eigen::Vector3d& lo0,
    const Eigen::Vector3d& hi0,
    const Eigen::Vector3d& lo1,
    const Eigen::Vector3d& hi1)
{
    for (int k = 0; k < 3; ++k) {
        if (hi0[k] < lo1[k] || hi1[k] < lo0[k]) return false;
    }
    return true;
}

} // namespace

TEST_CASE("per-primitive distance: closed forms in every region", "[envelope]")
{
    SECTION("triangle: interior, edge, vertex")
    {
        // The triangle (0,0,0), (1,0,0), (0,1,0): every foot is known by inspection.
        SampleEnvelope env(false);
        env.init(
            std::vector<Eigen::Vector3d>{
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector3d(1, 0, 0),
                Eigen::Vector3d(0, 1, 0)},
            std::vector<Eigen::Vector3i>{Eigen::Vector3i(0, 1, 2)},
            0.1);
        const Eigen::Vector3d x(1, 0, 0);
        check_closed_form(env, 0, Eigen::Vector3d(0.2, 0.2, 0.5), Eigen::Vector3d(0.2, 0.2, 0), 2);
        check_closed_form(
            env,
            0,
            Eigen::Vector3d(0.2, 0.3, -0.4),
            Eigen::Vector3d(0.2, 0.3, 0),
            2); // below: the same interior region
        check_closed_form(
            env,
            0,
            Eigen::Vector3d(0.5, -0.3, 0.4),
            Eigen::Vector3d(0.5, 0, 0),
            1,
            x);
        check_closed_form(
            env,
            0,
            Eigen::Vector3d(0.8, 0.8, 0.1),
            Eigen::Vector3d(0.5, 0.5, 0),
            1,
            Eigen::Vector3d(-1, 1, 0).normalized());
        check_closed_form(env, 0, Eigen::Vector3d(-0.3, -0.4, 0), Eigen::Vector3d(0, 0, 0), 0);
        check_closed_form(env, 0, Eigen::Vector3d(1.3, -0.1, 0.2), Eigen::Vector3d(1, 0, 0), 0);
        // On the triangle: d = 0, and the derivatives are reported as 0.
        Eigen::Vector3d g;
        Eigen::Matrix3d H;
        CHECK(env.primitive_distance(0, Eigen::Vector3d(0.25, 0.25, 0), &g, &H) == 0);
        CHECK(g.isZero(0));
        CHECK(H.isZero(0));
    }
    SECTION("segment: interior, endpoint; and a point primitive")
    {
        // Segment 0 from the origin to (2,0,0); entry 1 the isolated point (5,5,5).
        SampleEnvelope env(false);
        env.init(
            std::vector<Eigen::Vector3d>{
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector3d(2, 0, 0),
                Eigen::Vector3d(5, 5, 5)},
            std::vector<Eigen::Vector2i>{Eigen::Vector2i(0, 1), Eigen::Vector2i(2, 2)},
            0.1);
        const Eigen::Vector3d x(1, 0, 0);
        check_closed_form(env, 0, Eigen::Vector3d(1, 0.3, 0.4), Eigen::Vector3d(1, 0, 0), 1, x);
        check_closed_form(env, 0, Eigen::Vector3d(2.3, 0.4, 0), Eigen::Vector3d(2, 0, 0), 0);
        check_closed_form(env, 0, Eigen::Vector3d(-0.3, 0, -0.4), Eigen::Vector3d(0, 0, 0), 0);
        check_closed_form(env, 1, Eigen::Vector3d(5.3, 5, 5.4), Eigen::Vector3d(5, 5, 5), 0);
        Eigen::Vector3d g;
        Eigen::Matrix3d H;
        CHECK(env.primitive_distance(0, Eigen::Vector3d(1, 0, 0), &g, &H) == 0);
        CHECK(g.isZero(0));
        CHECK(H.isZero(0));
        CHECK(env.primitive_distance(1, Eigen::Vector3d(5, 5, 5), &g, &H) == 0);
        CHECK(g.isZero(0));
        CHECK(H.isZero(0));
    }
}

TEST_CASE("per-primitive triangle distance: closed forms", "[envelope]")
{
    using T = std::array<Eigen::Vector3d, 3>;
    SECTION("against a triangle")
    {
        SampleEnvelope env(false);
        env.init(
            std::vector<Eigen::Vector3d>{
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector3d(1, 0, 0),
                Eigen::Vector3d(0, 1, 0)},
            std::vector<Eigen::Vector3i>{Eigen::Vector3i(0, 1, 2)},
            0.1);
        // Parallel, 0.5 above and overlapping in projection: the closest pairs are interior
        // points of both, and the min is still found at a corner of the smaller one.
        const T above = {
            {Eigen::Vector3d(0.1, 0.1, 0.5),
             Eigen::Vector3d(0.4, 0.1, 0.5),
             Eigen::Vector3d(0.1, 0.4, 0.5)}};
        CHECK(env.primitive_triangle_distance(0, above) == Catch::Approx(0.5).margin(1e-14));
        // A vertical triangle piercing the interior: an edge of it crosses the primitive.
        const T pierce = {
            {Eigen::Vector3d(0.2, 0.2, -1),
             Eigen::Vector3d(0.3, 0.2, 1),
             Eigen::Vector3d(0.25, 0.3, 1)}};
        CHECK(env.primitive_triangle_distance(0, pierce) == 0);
        // Edge to edge: a triangle in the plane x = 0.5, beside the edge y = 0, whose edge
        // y = -0.3 is nearest at (0.5, -0.3, 0), interior to both edges. Its corners are farther
        // (sqrt(1.09) and more), and so are the primitive's (0.5).
        const T beside = {
            {Eigen::Vector3d(0.5, -0.3, -1),
             Eigen::Vector3d(0.5, -0.3, 1),
             Eigen::Vector3d(0.5, -2, 0)}};
        CHECK(env.primitive_triangle_distance(0, beside) == Catch::Approx(0.3).margin(1e-14));
    }
    SECTION("against a segment and a point")
    {
        SampleEnvelope env(false);
        env.init(
            std::vector<Eigen::Vector3d>{
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector3d(2, 0, 0),
                Eigen::Vector3d(5, 5, 5)},
            std::vector<Eigen::Vector2i>{Eigen::Vector2i(0, 1), Eigen::Vector2i(2, 2)},
            0.1);
        // The segment runs under the triangle, parallel, 0.5 below it.
        const T over = {
            {Eigen::Vector3d(0.5, -1, 0.5),
             Eigen::Vector3d(1.5, -1, 0.5),
             Eigen::Vector3d(1, 1, 0.5)}};
        CHECK(env.primitive_triangle_distance(0, over) == Catch::Approx(0.5).margin(1e-14));
        // The segment pierces a triangle in the plane x = 1.
        const T across = {
            {Eigen::Vector3d(1, -1, -1), Eigen::Vector3d(1, 1, -1), Eigen::Vector3d(1, 0, 1)}};
        CHECK(env.primitive_triangle_distance(0, across) == 0);
        // Beyond the endpoint (2,0,0): the triangle in the plane x = 2.5 around the axis.
        const T beyond = {
            {Eigen::Vector3d(2.5, -1, -1),
             Eigen::Vector3d(2.5, 1, -1),
             Eigen::Vector3d(2.5, 0, 1)}};
        CHECK(env.primitive_triangle_distance(0, beyond) == Catch::Approx(0.5).margin(1e-14));
        // The point (5,5,5) under a triangle at z = 5.5, and beside one's edge.
        const T roof = {
            {Eigen::Vector3d(4, 4, 5.5), Eigen::Vector3d(7, 4, 5.5), Eigen::Vector3d(4, 7, 5.5)}};
        CHECK(env.primitive_triangle_distance(1, roof) == Catch::Approx(0.5).margin(1e-14));
        const T wall = {
            {Eigen::Vector3d(5.3, 4, 4), Eigen::Vector3d(5.3, 6, 4), Eigen::Vector3d(5.3, 5, 4.6)}};
        CHECK(
            env.primitive_triangle_distance(1, wall) ==
            Catch::Approx(std::hypot(0.3, 0.4)).margin(1e-14));
    }
}

TEST_CASE("per-primitive distance derivatives match central differences", "[envelope]")
{
    SECTION("triangle, in each of its seven regions")
    {
        const std::vector<Eigen::Vector3d> V = skew_triangle_vertices();
        SampleEnvelope env(false);
        env.init(V, std::vector<Eigen::Vector3i>{Eigen::Vector3i(0, 1, 2)}, 0.1);
        const Eigen::Vector3d n = (V[1] - V[0]).cross(V[2] - V[0]).normalized();
        const Eigen::Vector3d centroid = (V[0] + V[1] + V[2]) / 3;
        for (const double h : {0.4, -0.25}) {
            const Eigen::Vector3d p = centroid + h * n;
            CHECK(region_rank(env, 0, p) == 0);
            check_derivatives(env, 0, p);
        }
        for (int i = 0; i < 3; ++i) {
            const Eigen::Vector3d& a = V[size_t(i)];
            const Eigen::Vector3d& b = V[size_t((i + 1) % 3)];
            const Eigen::Vector3d& c = V[size_t((i + 2) % 3)];
            // Outward in-plane normal of edge ab, away from c.
            Eigen::Vector3d out = (b - a).cross(n).normalized();
            if (out.dot(c - a) > 0) out = -out;
            const Eigen::Vector3d pe = 0.5 * (a + b) + 0.4 * out + 0.3 * n;
            CHECK(region_rank(env, 0, pe) == 1);
            check_derivatives(env, 0, pe);
            // Beyond vertex a, against both of its edges.
            const Eigen::Vector3d pv =
                a - 0.4 * ((b - a).normalized() + (c - a).normalized()).normalized() - 0.2 * n;
            CHECK(region_rank(env, 0, pv) == 2);
            check_derivatives(env, 0, pv);
        }
    }
    SECTION("segment interior, endpoints, point")
    {
        const std::vector<Eigen::Vector3d> V = {
            Eigen::Vector3d(0.1, 0.2, 0.3),
            Eigen::Vector3d(1.3, -0.1, 0.5),
            Eigen::Vector3d(0.4, 1.1, -0.2)};
        SampleEnvelope env(false);
        env.init(
            V,
            std::vector<Eigen::Vector2i>{Eigen::Vector2i(0, 1), Eigen::Vector2i(2, 2)},
            0.1);
        const Eigen::Vector3d e = (V[1] - V[0]).normalized();
        const Eigen::Vector3d side = e.cross(Eigen::Vector3d(0, 0, 1)).normalized();
        const Eigen::Vector3d pi = 0.3 * V[0] + 0.7 * V[1] + 0.4 * side + 0.2 * e.cross(side);
        CHECK(region_rank(env, 0, pi) == 1);
        check_derivatives(env, 0, pi);
        for (const Eigen::Vector3d& p :
             {Eigen::Vector3d(V[0] - 0.3 * e + 0.2 * side),
              Eigen::Vector3d(V[1] + 0.3 * e - 0.2 * side)}) {
            CHECK(region_rank(env, 0, p) == 2);
            check_derivatives(env, 0, p);
        }
        const Eigen::Vector3d pp = V[2] + Eigen::Vector3d(0.2, -0.3, 0.1);
        CHECK(region_rank(env, 1, pp) == 2);
        check_derivatives(env, 1, pp);
    }
}

TEST_CASE("primitives_in_box is every primitive whose box meets the query box", "[envelope]")
{
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> U(-0.1, 1.1), R(0, 0.15);
    const auto random_box = [&](Eigen::Vector3d& lo, Eigen::Vector3d& hi) {
        const Eigen::Vector3d c(U(rng), U(rng), U(rng));
        const Eigen::Vector3d r(R(rng), R(rng), R(rng));
        lo = c - r;
        hi = c + r;
    };
    // The brute force: each primitive's box from its vertices, compared closed.
    const auto brute = [](const std::vector<std::array<Eigen::Vector3d, 2>>& boxes,
                          const Eigen::Vector3d& lo,
                          const Eigen::Vector3d& hi) {
        std::vector<int64_t> ids;
        for (size_t i = 0; i < boxes.size(); ++i) {
            if (boxes_meet(boxes[i][0], boxes[i][1], lo, hi)) ids.push_back(int64_t(i));
        }
        return ids;
    };
    size_t empty = 0, nonempty = 0;
    const auto compare = [&](const SampleEnvelope& env,
                             const std::vector<std::array<Eigen::Vector3d, 2>>& boxes) {
        for (int k = 0; k < 300; ++k) {
            Eigen::Vector3d lo, hi;
            random_box(lo, hi);
            std::vector<int64_t> ids;
            env.primitives_in_box(lo, hi, ids);
            CHECK(ids == brute(boxes, lo, hi));
            ++(ids.empty() ? empty : nonempty);
        }
        // A box that is exactly one primitive's box finds it: the comparison is closed.
        std::vector<int64_t> ids;
        env.primitives_in_box(boxes[3][0], boxes[3][1], ids);
        CHECK(std::find(ids.begin(), ids.end(), 3) != ids.end());
    };

    SECTION("triangles")
    {
        std::vector<Eigen::Vector3d> V;
        std::vector<Eigen::Vector3i> F;
        random_triangles(rng, 400, V, F);
        SampleEnvelope env(false);
        env.init(V, F, 0.01);
        std::vector<std::array<Eigen::Vector3d, 2>> boxes;
        for (const auto& f : F) {
            boxes.push_back(
                {{V[f[0]].cwiseMin(V[f[1]]).cwiseMin(V[f[2]]),
                  V[f[0]].cwiseMax(V[f[1]]).cwiseMax(V[f[2]])}});
        }
        compare(env, boxes);
    }
    SECTION("segments and points")
    {
        std::vector<Eigen::Vector3d> V;
        std::vector<Eigen::Vector2i> E;
        random_segments_and_points(rng, 300, 100, V, E);
        SampleEnvelope env(false);
        env.init(V, E, 0.01);
        std::vector<std::array<Eigen::Vector3d, 2>> boxes;
        for (const auto& e : E) {
            boxes.push_back({{V[e[0]].cwiseMin(V[e[1]]), V[e[0]].cwiseMax(V[e[1]])}});
        }
        compare(env, boxes);
    }
    SECTION("2D segments and points, through the 2D overload")
    {
        std::vector<Eigen::Vector3d> V3;
        std::vector<Eigen::Vector2i> E;
        random_segments_and_points(rng, 300, 100, V3, E);
        std::vector<Eigen::Vector2d> V;
        for (const auto& v : V3) V.emplace_back(v[0], v[1]);
        SampleEnvelope env(false);
        env.init(V, E, 0.01);
        std::vector<std::array<Eigen::Vector2d, 2>> boxes;
        for (const auto& e : E) {
            boxes.push_back({{V[e[0]].cwiseMin(V[e[1]]), V[e[0]].cwiseMax(V[e[1]])}});
        }
        for (int k = 0; k < 300; ++k) {
            Eigen::Vector3d lo3, hi3;
            random_box(lo3, hi3);
            const Eigen::Vector2d lo = lo3.head<2>(), hi = hi3.head<2>();
            std::vector<int64_t> ids, expected;
            env.primitives_in_box(lo, hi, ids);
            for (size_t i = 0; i < boxes.size(); ++i) {
                if (!(boxes[i][1][0] < lo[0] || hi[0] < boxes[i][0][0] || boxes[i][1][1] < lo[1] ||
                      hi[1] < boxes[i][0][1])) {
                    expected.push_back(int64_t(i));
                }
            }
            CHECK(ids == expected);
            ++(ids.empty() ? empty : nonempty);
        }
        std::vector<int64_t> ids;
        env.primitives_in_box(boxes[3][0], boxes[3][1], ids);
        CHECK(std::find(ids.begin(), ids.end(), 3) != ids.end());
    }
    // Both answers come up, in every section: the comparison is not vacuous.
    CHECK(empty > 0);
    CHECK(nonempty > 0);
}

TEST_CASE("min over a box's primitives is the BVH's nearest distance", "[envelope]")
{
    // The box of radius sqrt(squared_distance(p)) around p holds every primitive at that distance,
    // so the min over it of primitive_distance is the min over ALL primitives -- exactly, which
    // checks that primitives_in_box loses none -- and is the BVH's distance up to rounding (its
    // foot point is computed with different arithmetic).
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> U(-0.3, 1.3);
    const auto check = [&](const SampleEnvelope& env, const size_t n_prim) {
        for (int k = 0; k < 300; ++k) {
            const Eigen::Vector3d p(U(rng), U(rng), U(rng));
            const double r = std::sqrt(env.squared_distance(p));
            const double pad = r * 1e-9 + 1e-12;
            std::vector<int64_t> ids;
            env.primitives_in_box(
                Eigen::Vector3d(p - Eigen::Vector3d::Constant(r + pad)),
                Eigen::Vector3d(p + Eigen::Vector3d::Constant(r + pad)),
                ids);
            REQUIRE(!ids.empty());
            double m_box = std::numeric_limits<double>::infinity();
            for (const int64_t id : ids) m_box = std::min(m_box, env.primitive_distance(id, p));
            double m_all = std::numeric_limits<double>::infinity();
            for (size_t id = 0; id < n_prim; ++id) {
                m_all = std::min(m_all, env.primitive_distance(int64_t(id), p));
            }
            CHECK(m_box == m_all);
            CHECK(m_box == Catch::Approx(r).margin(1e-12));
            Eigen::Vector3d q;
            CHECK(env.nearest_point(p, q) == Catch::Approx(m_box * m_box).margin(1e-12));
        }
    };
    SECTION("triangles")
    {
        std::vector<Eigen::Vector3d> V;
        std::vector<Eigen::Vector3i> F;
        random_triangles(rng, 200, V, F);
        SampleEnvelope env(false);
        env.init(V, F, 0.01);
        check(env, F.size());
    }
    SECTION("segments and points")
    {
        std::vector<Eigen::Vector3d> V;
        std::vector<Eigen::Vector2i> E;
        random_segments_and_points(rng, 150, 50, V, E);
        SampleEnvelope env(false);
        env.init(V, E, 0.01);
        check(env, E.size());
    }
}

TEST_CASE("per-primitive queries refuse what they cannot answer", "[envelope]")
{
    // Each refusal is a throw, not a crash: an uninitialized envelope has no BVH to dereference,
    // an id out of range would index past the input copies, and a query of the other dimension
    // would read a copy of the input that this envelope's init() never filled in.
    const std::array<Eigen::Vector3d, 3> tri = {
        {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0)}};
    const std::array<Eigen::Vector2d, 2> seg = {{Eigen::Vector2d(0, 1), Eigen::Vector2d(1, 1)}};
    const Eigen::Vector3d lo(0, 0, 0), hi(1, 1, 1);
    const Eigen::Vector2d lo2(0, 0), hi2(1, 1);
    std::vector<int64_t> ids;

    SECTION("an envelope no init() has filled in")
    {
        SampleEnvelope env(false);
        CHECK_THROWS(env.primitives_in_box(lo, hi, ids));
        CHECK_THROWS(env.primitive_distance(0, lo));
        CHECK_THROWS(env.primitive_triangle_distance(0, tri));
        CHECK_THROWS(env.primitives_in_box(lo2, hi2, ids));
        CHECK_THROWS(env.primitive_distance(0, lo2));
        CHECK_THROWS(env.primitive_segment_distance(0, seg));
    }
    SECTION("an id out of range")
    {
        SampleEnvelope env3(false);
        init_axis_3d(env3, 4.0, 0.1);
        CHECK_NOTHROW(env3.primitive_distance(0, lo));
        CHECK_THROWS(env3.primitive_distance(1, lo));
        CHECK_THROWS(env3.primitive_distance(-1, lo));
        CHECK_THROWS(env3.primitive_triangle_distance(1, tri));
        SampleEnvelope env2(false);
        init_axis_2d(env2, 4.0, 0.1);
        CHECK_NOTHROW(env2.primitive_distance(0, lo2));
        CHECK_THROWS(env2.primitive_distance(1, lo2));
        CHECK_THROWS(env2.primitive_distance(-1, lo2));
        CHECK_THROWS(env2.primitive_segment_distance(1, seg));
    }
    SECTION("a 2D envelope answers the 2D queries only")
    {
        SampleEnvelope env(false);
        init_axis_2d(env, 4.0, 0.1);
        env.primitives_in_box(Eigen::Vector2d(1, -1), Eigen::Vector2d(2, 1), ids);
        CHECK(ids == std::vector<int64_t>{0});
        CHECK_NOTHROW(env.primitive_distance(0, lo2));
        CHECK_NOTHROW(env.primitive_segment_distance(0, seg));
        CHECK_THROWS(env.primitives_in_box(lo, hi, ids));
        CHECK_THROWS(env.primitive_distance(0, lo));
        CHECK_THROWS(env.primitive_triangle_distance(0, tri));
    }
    SECTION("a 3D envelope answers the 3D queries only")
    {
        for (const bool triangles : {true, false}) {
            SampleEnvelope env(false);
            if (triangles) {
                env.init(
                    std::vector<Eigen::Vector3d>{tri[0], tri[1], tri[2]},
                    std::vector<Eigen::Vector3i>{Eigen::Vector3i(0, 1, 2)},
                    0.1);
            } else {
                init_axis_3d(env, 4.0, 0.1);
            }
            CHECK_NOTHROW(env.primitives_in_box(lo, hi, ids));
            CHECK_THROWS(env.primitives_in_box(lo2, hi2, ids));
            CHECK_THROWS(env.primitive_distance(0, lo2));
            CHECK_THROWS(env.primitive_segment_distance(0, seg));
        }
    }
}

// The 2D twins, on Edges2d envelopes: the same checks as the 3D cases above.

namespace {

void check_closed_form_2d(
    const SampleEnvelope& env,
    const int64_t id,
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& foot,
    const bool interior)
{
    Eigen::Vector2d g;
    Eigen::Matrix2d H;
    const double d = env.primitive_distance(id, p, &g, &H);
    CHECK(d == Catch::Approx((p - foot).norm()).margin(1e-14));
    CHECK((g - (p - foot).normalized()).norm() <= 1e-14);
    if (interior) {
        // d is affine across a segment's interior: no curvature, exactly.
        CHECK(H.isZero(0));
    } else {
        const Eigen::Vector2d u = (p - foot) / (p - foot).norm();
        const Eigen::Matrix2d expected = (Eigen::Matrix2d::Identity() - u * u.transpose()) / d;
        CHECK((H - expected).norm() <= 1e-13);
    }
}

void check_derivatives_2d(const SampleEnvelope& env, const int64_t id, const Eigen::Vector2d& p)
{
    const double h = 1e-6;
    Eigen::Vector2d g;
    Eigen::Matrix2d H;
    env.primitive_distance(id, p, &g, &H);
    for (int j = 0; j < 2; ++j) {
        Eigen::Vector2d pp = p, pm = p;
        pp[j] += h;
        pm[j] -= h;
        Eigen::Vector2d gp, gm;
        const double dp = env.primitive_distance(id, pp, &gp);
        const double dm = env.primitive_distance(id, pm, &gm);
        CHECK(std::abs((dp - dm) / (2 * h) - g[j]) <= 1e-6);
        for (int i = 0; i < 2; ++i) CHECK(std::abs((gp[i] - gm[i]) / (2 * h) - H(i, j)) <= 1e-5);
    }
}

/// n random 2D segments of length ~0.2 in the unit square, then m isolated points as (i, i).
void random_segments_and_points_2d(
    std::mt19937& rng,
    const int n,
    const int m,
    std::vector<Eigen::Vector2d>& V,
    std::vector<Eigen::Vector2i>& E)
{
    std::vector<Eigen::Vector3d> V3;
    random_segments_and_points(rng, n, m, V3, E);
    for (const auto& v : V3) V.emplace_back(v[0], v[1]);
}

} // namespace

TEST_CASE("per-primitive 2D distance: closed forms", "[envelope]")
{
    // Segment 0 from the origin to (2,0); entry 1 the isolated point (5,5).
    SampleEnvelope env(false);
    env.init(
        std::vector<Eigen::Vector2d>{
            Eigen::Vector2d(0, 0),
            Eigen::Vector2d(2, 0),
            Eigen::Vector2d(5, 5)},
        std::vector<Eigen::Vector2i>{Eigen::Vector2i(0, 1), Eigen::Vector2i(2, 2)},
        0.1);
    using S = std::array<Eigen::Vector2d, 2>;

    SECTION("point distance: interior, endpoints, point")
    {
        check_closed_form_2d(env, 0, Eigen::Vector2d(1.2, 0.5), Eigen::Vector2d(1.2, 0), true);
        check_closed_form_2d(env, 0, Eigen::Vector2d(0.7, -0.3), Eigen::Vector2d(0.7, 0), true);
        check_closed_form_2d(env, 0, Eigen::Vector2d(2.3, 0.4), Eigen::Vector2d(2, 0), false);
        check_closed_form_2d(env, 0, Eigen::Vector2d(-0.3, -0.4), Eigen::Vector2d(0, 0), false);
        check_closed_form_2d(env, 1, Eigen::Vector2d(5.3, 4.6), Eigen::Vector2d(5, 5), false);
        Eigen::Vector2d g;
        Eigen::Matrix2d H;
        CHECK(env.primitive_distance(0, Eigen::Vector2d(1, 0), &g, &H) == 0);
        CHECK(g.isZero(0));
        CHECK(H.isZero(0));
        CHECK(env.primitive_distance(1, Eigen::Vector2d(5, 5), &g, &H) == 0);
        CHECK(g.isZero(0));
        CHECK(H.isZero(0));
    }
    SECTION("segment distance")
    {
        // Crossing: exactly 0, from the orientation signs.
        CHECK(env.primitive_segment_distance(0, S{{{1, -1}, {1.5, 1}}}) == 0);
        // Touching without crossing -- an endpoint on the segment, and collinear overlap -- is an
        // endpoint distance at 0.
        CHECK(env.primitive_segment_distance(0, S{{{1, 0}, {1, 1}}}) == 0);
        CHECK(env.primitive_segment_distance(0, S{{{1.5, 0}, {3, 0}}}) == 0);
        // Parallel above, overlapping in x: 0.5. Collinear past the end: 0.25.
        CHECK(
            env.primitive_segment_distance(0, S{{{0.5, 0.5}, {3, 0.5}}}) ==
            Catch::Approx(0.5).margin(1e-14));
        CHECK(
            env.primitive_segment_distance(0, S{{{2.25, 0}, {3, 0}}}) ==
            Catch::Approx(0.25).margin(1e-14));
        // Beyond the endpoint (2,0), the query's interior nearest: a vertical segment at x = 2.4.
        CHECK(
            env.primitive_segment_distance(0, S{{{2.4, -1}, {2.4, 1}}}) ==
            Catch::Approx(0.4).margin(1e-14));
        // The point (5,5) beside a segment's interior, and beyond its end.
        CHECK(
            env.primitive_segment_distance(1, S{{{4, 5.3}, {6, 5.3}}}) ==
            Catch::Approx(0.3).margin(1e-14));
        CHECK(
            env.primitive_segment_distance(1, S{{{5.3, 5.4}, {6, 6}}}) ==
            Catch::Approx(0.5).margin(1e-14));
    }
}

TEST_CASE("per-primitive 2D distance derivatives match central differences", "[envelope]")
{
    const std::vector<Eigen::Vector2d> V = {
        Eigen::Vector2d(0.1, 0.2),
        Eigen::Vector2d(1.3, -0.1),
        Eigen::Vector2d(0.4, 1.1)};
    SampleEnvelope env(false);
    env.init(V, std::vector<Eigen::Vector2i>{Eigen::Vector2i(0, 1), Eigen::Vector2i(2, 2)}, 0.1);
    const Eigen::Vector2d e = (V[1] - V[0]).normalized();
    const Eigen::Vector2d n(-e[1], e[0]);
    for (const Eigen::Vector2d& p :
         {Eigen::Vector2d(0.3 * V[0] + 0.7 * V[1] + 0.4 * n),
          Eigen::Vector2d(0.6 * V[0] + 0.4 * V[1] - 0.2 * n),
          Eigen::Vector2d(V[0] - 0.3 * e + 0.2 * n),
          Eigen::Vector2d(V[1] + 0.3 * e - 0.2 * n)}) {
        check_derivatives_2d(env, 0, p);
    }
    check_derivatives_2d(env, 1, V[2] + Eigen::Vector2d(0.2, -0.3));
    // Inside a skew segment, where I - u u^T - e e^T would be 0 only up to rounding, the
    // Hessian is the zero matrix exactly.
    for (const double s : {0.4, -0.2}) {
        Eigen::Matrix2d H;
        env.primitive_distance(0, Eigen::Vector2d(0.5 * (V[0] + V[1]) + s * n), nullptr, &H);
        CHECK(H.isZero(0));
    }
}

TEST_CASE("min over a 2D box's primitives is the BVH's nearest distance", "[envelope]")
{
    // As in 3D: the min over the box's primitives is the min over all of them exactly, and the
    // BVH's own distance up to rounding.
    std::mt19937 rng(9);
    std::vector<Eigen::Vector2d> V;
    std::vector<Eigen::Vector2i> E;
    random_segments_and_points_2d(rng, 150, 50, V, E);
    SampleEnvelope env(false);
    env.init(V, E, 0.01);
    std::uniform_real_distribution<double> U(-0.3, 1.3);
    for (int k = 0; k < 300; ++k) {
        const Eigen::Vector2d p(U(rng), U(rng));
        const double r = std::sqrt(env.squared_distance(p));
        const double pad = r * 1e-9 + 1e-12;
        std::vector<int64_t> ids;
        env.primitives_in_box(
            Eigen::Vector2d(p - Eigen::Vector2d::Constant(r + pad)),
            Eigen::Vector2d(p + Eigen::Vector2d::Constant(r + pad)),
            ids);
        REQUIRE(!ids.empty());
        double m_box = std::numeric_limits<double>::infinity();
        for (const int64_t id : ids) m_box = std::min(m_box, env.primitive_distance(id, p));
        double m_all = std::numeric_limits<double>::infinity();
        for (size_t id = 0; id < E.size(); ++id) {
            m_all = std::min(m_all, env.primitive_distance(int64_t(id), p));
        }
        CHECK(m_box == m_all);
        CHECK(m_box == Catch::Approx(r).margin(1e-12));
        Eigen::Vector2d q;
        CHECK(env.nearest_point(p, q) == Catch::Approx(m_box * m_box).margin(1e-12));
    }
}
