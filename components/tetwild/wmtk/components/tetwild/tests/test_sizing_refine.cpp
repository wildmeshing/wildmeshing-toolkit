#include <wmtk/TetMesh.h>
#include <wmtk/components/tetwild/TetWildMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::tetwild;

// Two disjoint tets: tet 0 = {0,1,2,3} near the origin, tet 1 = {4,5,6,7} far
// away. Because the tets share no vertices, a ring-BFS from one never reaches
// the other, giving a clean "outside the region is untouched" check.
namespace {

void build_two_tets(TetWildMesh& mesh)
{
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}, {{4, 5, 6, 7}}};
    mesh.init(8, tets);

    std::vector<VertexAttributes> va(8);
    const Vector3d base0(0, 0, 0), base1(10, 0, 0);
    const std::array<Vector3d, 4> corner = {
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 1, 0),
        Vector3d(0, 0, 1)};
    for (int k = 0; k < 4; ++k) {
        va[k].m_posf = base0 + corner[k];
        va[k + 4].m_posf = base1 + corner[k];
    }
    for (int i = 0; i < 8; ++i) {
        va[i].m_is_rounded = true;
        va[i].m_pos = to_rational(va[i].m_posf);
        va[i].m_sizing_scalar = 1.0;
    }
    std::vector<TetAttributes> ta(2);
    mesh.create_mesh_attributes(va, ta);
}

double sz(TetWildMesh& mesh, size_t v)
{
    return mesh.m_vertex_attribute[v].m_sizing_scalar;
}

} // namespace

TEST_CASE("stuck-refine-region-and-factor", "[tetwild_operation][stuck_refine]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(12, 2, 2));
    params.stuck_refine_num_worst = 1;
    params.stuck_refine_rings = 3;
    params.stuck_refine_factor = 0.5;
    params.stuck_refine_min_scalar = 0.1;
    params.stuck_refine_gradation = 1.0; // disable smoothing to isolate the refinement

    SampleEnvelope env;
    TetWildMesh mesh(params, env, 1);
    build_two_tets(mesh);
    // Make tet 0 the single worst element.
    mesh.m_tet_attribute[0].m_quality = 1e40;
    mesh.m_tet_attribute[1].m_quality = 30.0;

    const size_t n = mesh.refine_sizing_around_worst();
    CHECK(n == 4); // exactly tet 0's four vertices

    // tet 0's vertices halved; tet 1's vertices untouched.
    for (size_t v : {0, 1, 2, 3}) CHECK(sz(mesh, v) == 0.5);
    for (size_t v : {4, 5, 6, 7}) CHECK(sz(mesh, v) == 1.0);

    // Force-split queued tet 0's single longest edge (a unit-tet diagonal).
    CHECK(mesh.m_force_split_edges.size() == 1);
}

TEST_CASE("stuck-refine-floor-clamp", "[tetwild_operation][stuck_refine]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(12, 2, 2));
    params.stuck_refine_num_worst = 1;
    params.stuck_refine_rings = 3;
    params.stuck_refine_factor = 0.5;
    params.stuck_refine_min_scalar = 0.7; // floor above factor*1.0 => clamps
    params.stuck_refine_gradation = 1.0;

    SampleEnvelope env;
    TetWildMesh mesh(params, env, 1);
    build_two_tets(mesh);
    mesh.m_tet_attribute[0].m_quality = 1e40;
    mesh.m_tet_attribute[1].m_quality = 30.0;

    mesh.refine_sizing_around_worst();
    for (size_t v : {0, 1, 2, 3}) CHECK(sz(mesh, v) == 0.7); // max(0.7, 0.5)
    for (size_t v : {4, 5, 6, 7}) CHECK(sz(mesh, v) == 1.0);
}

TEST_CASE("stuck-refine-num-worst", "[tetwild_operation][stuck_refine]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(12, 2, 2));
    params.stuck_refine_num_worst = 2; // both tets are "worst"
    params.stuck_refine_rings = 0; // only the tets' own vertices
    params.stuck_refine_factor = 0.5;
    params.stuck_refine_min_scalar = 0.01;
    params.stuck_refine_gradation = 1.0;

    SampleEnvelope env;
    TetWildMesh mesh(params, env, 1);
    build_two_tets(mesh);
    mesh.m_tet_attribute[0].m_quality = 1e40;
    mesh.m_tet_attribute[1].m_quality = 1e30;

    const size_t n = mesh.refine_sizing_around_worst();
    CHECK(n == 8); // all vertices of both worst tets
    for (size_t v = 0; v < 8; ++v) CHECK(sz(mesh, v) == 0.5);
    // Each worst tet queues its own longest edge for force-split.
    CHECK(mesh.m_force_split_edges.size() == 2);
}

TEST_CASE("stuck-refine-gradation-monotone", "[tetwild_operation][stuck_refine]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(12, 2, 2));

    SampleEnvelope env;
    TetWildMesh mesh(params, env, 1);
    build_two_tets(mesh);

    // Seed vertex 0 (in tet 0) with a small sizing; everything else at 1.
    mesh.m_vertex_attribute[0].m_sizing_scalar = 0.1;
    std::array<double, 8> before;
    for (size_t v = 0; v < 8; ++v) before[v] = sz(mesh, v);

    mesh.gradation_smooth_sizing(2.0, {0});

    // (a) monotone: nothing increased.
    for (size_t v = 0; v < 8; ++v) CHECK(sz(mesh, v) <= before[v]);
    // (b) seed unchanged.
    CHECK(sz(mesh, 0) == 0.1);
    // (c) the seed's tet-0 neighbors are capped at grade * seed = 0.2.
    for (size_t v : {1, 2, 3}) CHECK(sz(mesh, v) <= 0.2 + 1e-12);
    // (d) the disconnected tet 1 is untouched.
    for (size_t v : {4, 5, 6, 7}) CHECK(sz(mesh, v) == 1.0);
    // (e) gradation satisfied on every edge of tet 0 (both directions).
    const std::array<size_t, 4> t0 = {0, 1, 2, 3};
    for (size_t a : t0)
        for (size_t b : t0)
            if (a != b) CHECK(sz(mesh, a) <= 2.0 * sz(mesh, b) + 1e-12);
}

TEST_CASE("stuck-refine-disabled-noop", "[tetwild_operation][stuck_refine]")
{
    // grade <= 1 disables the smoothing entirely.
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(12, 2, 2));
    SampleEnvelope env;
    TetWildMesh mesh(params, env, 1);
    build_two_tets(mesh);
    mesh.m_vertex_attribute[0].m_sizing_scalar = 0.1;
    mesh.gradation_smooth_sizing(1.0, {0});
    CHECK(sz(mesh, 1) == 1.0);
    CHECK(sz(mesh, 2) == 1.0);
    CHECK(sz(mesh, 3) == 1.0);
}

TEST_CASE("skip-good-regions-active-vertices", "[tetwild_operation][skip_good_regions]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(12, 2, 2));
    params.stop_energy = 100;
    params.skip_good_regions_margin = 0.9; // active energy >= 90 -> m_quality >= 90^3 = 729000

    SampleEnvelope env;
    TetWildMesh mesh(params, env, 1);
    build_two_tets(mesh);

    SECTION("only the bad tet's vertices are active")
    {
        mesh.m_tet_attribute[0].m_quality = 1e9; // energy 1000 >= 90 -> active
        mesh.m_tet_attribute[1].m_quality = 1000; // energy 10 < 90 -> good
        auto av = mesh.active_vertices();
        std::sort(av.begin(), av.end());
        CHECK(av == std::vector<size_t>{0, 1, 2, 3});
    }
    SECTION("all-good -> no active vertices")
    {
        mesh.m_tet_attribute[0].m_quality = 1000;
        mesh.m_tet_attribute[1].m_quality = 1000;
        CHECK(mesh.active_vertices().empty());
    }
    SECTION("both bad -> every vertex active")
    {
        mesh.m_tet_attribute[0].m_quality = 1e9;
        mesh.m_tet_attribute[1].m_quality = 1e9;
        CHECK(mesh.active_vertices().size() == 8);
    }
}
