// Containment queries that only the exact envelope can answer.
//
// SampleEnvelope has two backends. The sampled one places points along the query and asks the
// BVH whether each is within eps of the input; it cannot see anything that happens between
// two samples, and pays for that by shrinking its acceptance radius (eps/sqrt(3) for a
// triangle, eps/2 for a segment -- see the derivations on SampleEnvelope::eps2 and eps2_edge).
// The exact one asks whether the query is covered by the union of the eps-neighbourhoods of
// the input primitives and decides that with exact predicates, so it uses the full eps.
//
// Until fast-envelope gained envelopes built from edges, only the triangle-mesh envelope had
// an exact backend; the segment and 2D queries threw. These cases pin what the exact path now
// answers, and in particular the two places where the two backends legitimately disagree:
// the band between the shrunk radius and eps, where sampled is stricter, and a gap narrower
// than the sample spacing, where sampled is wrong.
#include <catch2/catch_test_macros.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <cmath>

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
    // The sampled test accepts a segment only when every sample is within eps/2, because the
    // spacing guarantees no more than another eps/2 between samples. So above eps/2 it starts
    // rejecting geometry the input actually contains. That is safe -- it errs towards
    // rejection -- but it is the reason the exact envelope is worth having.
    //
    // The offset below has to sit in the band where the two disagree. The upper end is not
    // eps: fast-envelope wraps each segment in a hexahedron of half-width eps/sqrt(3), whose
    // corners reach eps but whose inscribed sphere is eps/sqrt(3). Staying inside that sphere
    // is inside the hexahedron whatever orientation the frame comes out with, which keeps this
    // from depending on how seg_cube happens to pick its two cross-section axes.
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
