#include <wmtk/TriMesh.h>
#include <wmtk/components/triwild/TriWildMesh.h>
#include <wmtk/Types.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::triwild;

// A feature point is a 0-dimensional feature of the curve network -- an open polyline's
// endpoint, or a junction. collapse_breaks_feature is the policy that keeps one from being
// dropped or displaced by an edge collapse, which is what used to delete open polylines
// outright: a polyline erodes into its own tip until a single segment is left, and that
// segment carries a feature at BOTH ends, which the surface order test does not refuse.
//
// The policy is a pure function of two vertices' attributes, so it is testable without
// building a real curve network or running an operation.
namespace {

// One triangle is enough: the policy only reads m_feature_id and m_posf.
void build_one_tri(TriWildMesh& mesh, const std::array<Vector2d, 3>& p)
{
    const std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
    mesh.init(3, tris);
    mesh.m_vertex_attribute.resize(3);
    mesh.m_edge_attribute.resize(3);
    mesh.m_face_attribute.resize(1);
    for (int i = 0; i < 3; ++i) {
        auto& va = mesh.m_vertex_attribute[i];
        va.m_posf = p[i];
        va.m_pos = to_rational(p[i]);
        va.m_is_rounded = true;
    }
}

Parameters make_params()
{
    Parameters params;
    params.init(Vector2d(0, 0), Vector2d(10, 10));
    return params;
}

} // namespace

TEST_CASE("triwild-feature-collapse-policy", "[triwild_operation][feature_points]")
{
    Parameters params = make_params();
    const double eps = 1.0; // envelope_eps used directly, so the numbers below are readable

    // v0 at the origin carries feature 0, anchored exactly where it sits.
    // v1 sits 0.5 away (inside eps), v2 sits 5 away (well outside).
    TriWildMesh mesh(params, eps, 0);
    build_one_tri(mesh, {Vector2d(0, 0), Vector2d(0.5, 0), Vector2d(5, 0)});
    mesh.m_feature_points = {Vector2d(0, 0)};
    mesh.m_vertex_extra[0].m_feature_id = 0;

    SECTION("a vertex carrying no feature is never blocked")
    {
        CHECK_FALSE(mesh.collapse_breaks_feature(1, 2));
        CHECK_FALSE(mesh.collapse_breaks_feature(2, 1));
    }

    SECTION("collapsing a feature into a nearby vertex is allowed")
    {
        // v1 is within eps of feature 0, so the feature stays represented.
        CHECK_FALSE(mesh.collapse_breaks_feature(0, 1));
    }

    SECTION("collapsing a feature into a distant vertex is refused")
    {
        // This is the case that deletes a polyline: the anchor would be left behind.
        CHECK(mesh.collapse_breaks_feature(0, 2));
    }

    SECTION("collapsing INTO a feature is always fine")
    {
        // v2 carries nothing; the feature on v0 is untouched by v2 disappearing.
        CHECK_FALSE(mesh.collapse_breaks_feature(2, 0));
    }

    SECTION("two features closer than eps may merge -- the survivor covers both")
    {
        // v1 stands for its own anchor 0.5 away, inside eps. Collapsing either way leaves a
        // vertex within eps of BOTH anchors, so nothing is lost and the merge is allowed.
        // Refusing it outright (an earlier version did) deadlocks the mesh: on the puzzle
        // integration model it stranded a degenerate triangle and the max energy stayed at
        // 1e50 for all 82 iterations instead of converging in 3.
        mesh.m_feature_points.push_back(Vector2d(0.5, 0));
        mesh.m_vertex_extra[1].m_feature_id = 1;
        CHECK_FALSE(mesh.collapse_breaks_feature(0, 1));
        CHECK_FALSE(mesh.collapse_breaks_feature(1, 0));
    }

    SECTION("two features further apart than eps still cannot merge")
    {
        // v2 stands for an anchor 5 away. Collapsing v0 into it would leave feature 0 with no
        // vertex within eps, and the plain distance test catches that without needing a
        // special case for "v2 already carries something".
        mesh.m_feature_points.push_back(Vector2d(5, 0));
        mesh.m_vertex_extra[2].m_feature_id = 1;
        CHECK(mesh.collapse_breaks_feature(0, 2));
        CHECK(mesh.collapse_breaks_feature(2, 0));
    }

    SECTION("a vertex already carrying the SAME feature is fine")
    {
        mesh.m_vertex_extra[1].m_feature_id = 0;
        CHECK_FALSE(mesh.collapse_breaks_feature(0, 1));
    }

    SECTION("protection does not depend on rounding")
    {
        // The pre-existing order test is gated on m_is_rounded, so an un-rounded endpoint
        // slipped through it. This rule is not.
        mesh.m_vertex_attribute[0].m_is_rounded = false;
        CHECK(mesh.collapse_breaks_feature(0, 2));
    }

    SECTION("preserve_feature_points off restores the old behaviour")
    {
        params.preserve_feature_points = false;
        TriWildMesh off(params, eps, 0);
        build_one_tri(off, {Vector2d(0, 0), Vector2d(0.5, 0), Vector2d(5, 0)});
        off.m_feature_points = {Vector2d(0, 0)};
        off.m_vertex_extra[0].m_feature_id = 0;
        CHECK_FALSE(off.collapse_breaks_feature(0, 2));
    }
}

TEST_CASE("triwild-feature-smoothing-is-a-ball-not-a-pin", "[triwild_operation][feature_points]")
{
    Parameters params = make_params();
    const double eps = 1.0;
    TriWildMesh mesh(params, eps, 0);
    build_one_tri(mesh, {Vector2d(0, 0), Vector2d(0.5, 0), Vector2d(5, 0)});
    mesh.m_feature_points = {Vector2d(0, 0)};
    mesh.m_vertex_extra[0].m_feature_id = 0;

    // A feature vertex is free to move, just not away: smoothing keeps whatever quality it
    // can find inside the ball. Freezing it outright would be simpler and worse.
    CHECK(mesh.smoothing_position_is_allowed(0, Vector2d(0, 0)));
    CHECK(mesh.smoothing_position_is_allowed(0, Vector2d(0.9, 0)));
    CHECK(mesh.smoothing_position_is_allowed(0, Vector2d(0, -0.9)));
    CHECK(mesh.smoothing_position_is_allowed(0, Vector2d(0.7, 0.7))); // |.| = 0.99
    CHECK_FALSE(mesh.smoothing_position_is_allowed(0, Vector2d(1.1, 0)));
    CHECK_FALSE(mesh.smoothing_position_is_allowed(0, Vector2d(0.8, 0.8))); // |.| = 1.13

    // A vertex with no feature is unconstrained by this rule.
    CHECK(mesh.smoothing_position_is_allowed(1, Vector2d(100, 100)));
}

TEST_CASE("triwild-feature-retention-accounting", "[triwild_operation][feature_points]")
{
    Parameters params = make_params();
    const double eps = 1.0;
    TriWildMesh mesh(params, eps, 0);
    build_one_tri(mesh, {Vector2d(0, 0), Vector2d(0.5, 0), Vector2d(5, 0)});
    mesh.m_feature_points = {Vector2d(0, 0), Vector2d(5, 0)};

    SECTION("anchors that still have a vertex on them are retained")
    {
        // v0 is at (0,0) = feature 0, v2 at (5,0) = feature 1.
        const auto [kept, total] = mesh.feature_retention();
        CHECK(total == 2);
        CHECK(kept == 2);
    }

    SECTION("covered by a vertex that does not carry the id still counts")
    {
        // Retention is geometric on purpose: after a legitimate merge the surviving vertex
        // carries one id but covers both anchors, and counting ids would under-report it.
        // v0 sits exactly on feature 0 and v2 exactly on feature 1, carrying neither.
        const auto [kept, total] = mesh.feature_retention();
        CHECK(kept == 2);
        CHECK(total == 2);
    }

    SECTION("an anchor with no vertex near it is not retained")
    {
        mesh.m_feature_points.push_back(Vector2d(50, 50)); // nothing is within eps of this
        const auto [kept, total] = mesh.feature_retention();
        CHECK(total == 3);
        CHECK(kept == 2);
    }
}
