#include <wmtk/TriMesh.h>
#include <wmtk/components/triwild/TriWildMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::triwild;

// Two disjoint triangles: face 0 = {0,1,2} near the origin, face 1 = {3,4,5} far away.
// Because they share no vertices, a ring-BFS from one never reaches the other, which gives
// a clean "outside the region is untouched" check. Mirrors the tetwild test of the same
// name, so the two stay comparable.
namespace {

void build_two_tris(TriWildMesh& mesh)
{
    const std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{3, 4, 5}}};
    mesh.init(6, tris);

    mesh.m_vertex_attribute.resize(6);
    mesh.m_edge_attribute.resize(3 * tris.size());
    mesh.m_face_attribute.resize(tris.size());

    const Vector2d base0(0, 0), base1(10, 0);
    const std::array<Vector2d, 3> corner = {Vector2d(0, 0), Vector2d(1, 0), Vector2d(0, 1)};
    for (int k = 0; k < 3; ++k) {
        mesh.m_vertex_attribute[k].m_posf = base0 + corner[k];
        mesh.m_vertex_attribute[k + 3].m_posf = base1 + corner[k];
    }
    for (int i = 0; i < 6; ++i) {
        auto& va = mesh.m_vertex_attribute[i];
        va.m_is_rounded = true;
        va.m_pos = to_rational(va.m_posf);
        va.m_sizing_scalar = 1.0;
    }
}

double sz(const TriWildMesh& mesh, size_t v)
{
    return mesh.m_vertex_attribute[v].m_sizing_scalar;
}

Parameters make_params()
{
    Parameters params;
    params.init(Vector2d(-1, -1), Vector2d(12, 2));
    return params;
}

} // namespace

TEST_CASE("triwild-stuck-refine-region-and-factor", "[triwild_operation][stuck_refine]")
{
    Parameters params = make_params();
    params.stuck_refine_num_worst = 1;
    params.stuck_refine_rings = 3;
    params.stuck_refine_factor = 0.5;
    params.stuck_refine_min_scalar = 0.1;
    params.stuck_refine_gradation = 1.0; // disable smoothing to isolate the refinement
    params.stuck_refine_force_split = true;

    TriWildMesh mesh(params, params.eps, 0);
    build_two_tris(mesh);
    // Make face 0 the single worst element.
    mesh.m_face_attribute[0].m_quality = 1e40;
    mesh.m_face_attribute[1].m_quality = 3.0;

    const size_t n = mesh.refine_sizing_around_worst(0);
    CHECK(n == 3); // exactly face 0's three vertices

    // face 0's vertices halved; face 1's vertices untouched.
    for (size_t v : {0, 1, 2}) CHECK(sz(mesh, v) == 0.5);
    for (size_t v : {3, 4, 5}) CHECK(sz(mesh, v) == 1.0);

    // Force-split queued face 0's single longest edge (the hypotenuse (1,2)).
    REQUIRE(mesh.m_force_split_edges.size() == 1);
    CHECK(mesh.is_force_split_edge(1, 2));
    CHECK(mesh.is_force_split_edge(2, 1)); // the lookup is unordered
    CHECK_FALSE(mesh.is_force_split_edge(0, 1));
}

TEST_CASE("triwild-stuck-refine-floor-clamp", "[triwild_operation][stuck_refine]")
{
    Parameters params = make_params();
    params.stuck_refine_num_worst = 1;
    params.stuck_refine_rings = 3;
    params.stuck_refine_factor = 0.5;
    params.stuck_refine_min_scalar = 0.7; // floor above factor*1.0 => clamps
    params.stuck_refine_gradation = 1.0;

    TriWildMesh mesh(params, params.eps, 0);
    build_two_tris(mesh);
    mesh.m_face_attribute[0].m_quality = 1e40;
    mesh.m_face_attribute[1].m_quality = 3.0;

    mesh.refine_sizing_around_worst(0);
    for (size_t v : {0, 1, 2}) CHECK(sz(mesh, v) == 0.7); // max(0.7, 0.5)
    for (size_t v : {3, 4, 5}) CHECK(sz(mesh, v) == 1.0);
}

TEST_CASE("triwild-stuck-refine-num-worst", "[triwild_operation][stuck_refine]")
{
    Parameters params = make_params();
    params.stuck_refine_num_worst = 2; // both faces are "worst"
    params.stuck_refine_rings = 0; // only the faces' own vertices
    params.stuck_refine_factor = 0.5;
    params.stuck_refine_min_scalar = 0.01;
    params.stuck_refine_gradation = 1.0;

    TriWildMesh mesh(params, params.eps, 0);
    build_two_tris(mesh);
    mesh.m_face_attribute[0].m_quality = 1e40;
    mesh.m_face_attribute[1].m_quality = 1e30;

    const size_t n = mesh.refine_sizing_around_worst(0);
    CHECK(n == 6); // all vertices of both worst faces
    for (size_t v = 0; v < 6; ++v) CHECK(sz(mesh, v) == 0.5);
    // Each worst face queues its own longest edge for force-split.
    CHECK(mesh.m_force_split_edges.size() == 2);
}

TEST_CASE("triwild-stuck-refine-energy-is-not-cubed", "[triwild_operation][stuck_refine]")
{
    // The 2D quality *is* the AMIPS2D energy, so the filter compares it directly against
    // filter_energy = max(max_energy / 100, stop_energy). tetwild cube-roots its quality
    // first; getting that wrong here would let far-too-good faces be selected.
    Parameters params = make_params();
    params.stop_energy = 10;
    params.stuck_refine_num_worst = 0; // everything above the filter energy
    params.stuck_refine_rings = 0;
    params.stuck_refine_gradation = 1.0;

    TriWildMesh mesh(params, params.eps, 0);
    build_two_tris(mesh);
    mesh.m_face_attribute[0].m_quality = 20.0; // above stop_energy => selected
    mesh.m_face_attribute[1].m_quality = 5.0; // below stop_energy => not selected

    const size_t n = mesh.refine_sizing_around_worst(0);
    CHECK(n == 3);
    for (size_t v : {0, 1, 2}) CHECK(sz(mesh, v) < 1.0);
    for (size_t v : {3, 4, 5}) CHECK(sz(mesh, v) == 1.0);
}

TEST_CASE("triwild-stuck-refine-gradation-monotone", "[triwild_operation][stuck_refine]")
{
    Parameters params = make_params();

    TriWildMesh mesh(params, params.eps, 0);
    build_two_tris(mesh);

    // Seed vertex 0 (in face 0) with a small sizing; everything else at 1.
    mesh.m_vertex_attribute[0].m_sizing_scalar = 0.1;
    std::array<double, 6> before;
    for (size_t v = 0; v < 6; ++v) before[v] = sz(mesh, v);

    mesh.gradation_smooth_sizing(2.0, {0});

    // (a) monotone: nothing increased.
    for (size_t v = 0; v < 6; ++v) CHECK(sz(mesh, v) <= before[v]);
    // (b) seed unchanged.
    CHECK(sz(mesh, 0) == 0.1);
    // (c) the seed's face-0 neighbors are capped at grade * seed = 0.2.
    for (size_t v : {1, 2}) CHECK(sz(mesh, v) <= 0.2 + 1e-12);
    // (d) the disconnected face 1 is untouched.
    for (size_t v : {3, 4, 5}) CHECK(sz(mesh, v) == 1.0);
    // (e) gradation satisfied on every edge of face 0 (both directions).
    for (size_t a : {0, 1, 2})
        for (size_t b : {0, 1, 2})
            if (a != b) CHECK(sz(mesh, a) <= 2.0 * sz(mesh, b) + 1e-12);
}

TEST_CASE("triwild-stuck-refine-disabled-noop", "[triwild_operation][stuck_refine]")
{
    // grade <= 1 disables the smoothing entirely.
    Parameters params = make_params();
    TriWildMesh mesh(params, params.eps, 0);
    build_two_tris(mesh);
    mesh.m_vertex_attribute[0].m_sizing_scalar = 0.1;
    mesh.gradation_smooth_sizing(1.0, {0});
    CHECK(sz(mesh, 1) == 1.0);
    CHECK(sz(mesh, 2) == 1.0);
}

TEST_CASE("triwild-skip-good-regions-active-vertices", "[triwild_operation][skip_good_regions]")
{
    Parameters params = make_params();
    params.stop_energy = 10;
    params.skip_good_regions_margin = 0.9; // active threshold = 9

    TriWildMesh mesh(params, params.eps, 0);
    build_two_tris(mesh);
    mesh.m_face_attribute[0].m_quality = 12.0; // active
    mesh.m_face_attribute[1].m_quality = 4.0; // good, skipped

    CHECK(mesh.active_quality_threshold() == 9.0);

    auto active = mesh.active_vertices();
    std::sort(active.begin(), active.end());
    CHECK(active == std::vector<size_t>{0, 1, 2});

    // Below the threshold on both faces => nothing to smooth.
    mesh.m_face_attribute[0].m_quality = 4.0;
    CHECK(mesh.active_vertices().empty());
}
