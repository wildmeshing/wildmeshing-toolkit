#include <wmtk/TetMesh.h>
#include <wmtk/components/tetwild/TetWildMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::tetwild;

// ---------------------------------------------------------------------------
// Minimal fixture for the surface 3->2 edge flip.
//
// Five vertices: a=0, b=1 (the swapped edge), and a ring c=2, d=3, e=4. The
// three tets share edge (a,b):
//     T0 = (a,b,c,d)   T1 = (a,b,d,e)   T2 = (a,b,e,c)
// a,b sit on the +/-z axis, c,d,e form an equilateral triangle in z=0, so all
// three tets are positively oriented and non-degenerate.
//
// A 3->2 swap of edge (a,b) removes it, produces tets (a,c,d,e),(b,c,d,e) and
// the new face (c,d,e). When (a,b,c),(a,b,d) are the surface faces, the flip is
// the surface diagonal flip (a,b) -> (c,d): surface becomes (a,c,d),(b,c,d).
// ---------------------------------------------------------------------------
namespace {

constexpr size_t A = 0, B = 1, C = 2, D = 3, E = 4;

// Build the ring mesh; a,b,c,d are on the surface, e is interior. Faces are
// left untagged (the caller sets m_is_surface_fs).
void build_ring(TetWildMesh& mesh)
{
    std::vector<std::array<size_t, 4>> tets = {{{A, B, C, D}}, {{A, B, D, E}}, {{A, B, E, C}}};
    mesh.init(5, tets);

    std::vector<VertexAttributes> va(5);
    va[A].m_posf = Vector3d(0, 0, -1);
    va[B].m_posf = Vector3d(0, 0, 1);
    va[C].m_posf = Vector3d(1, 0, 0);
    va[D].m_posf = Vector3d(-0.5, 0.8660254037844386, 0);
    va[E].m_posf = Vector3d(-0.5, -0.8660254037844386, 0);
    for (int i = 0; i < 5; ++i) {
        va[i].m_is_rounded = true;
        va[i].m_pos = to_rational(va[i].m_posf);
        va[i].m_is_on_open_boundary = false;
    }
    for (size_t i : {A, B, C, D}) {
        va[i].m_is_on_surface = true;
        va[i].m_order = 1;
    }
    va[E].m_is_on_surface = false;
    va[E].m_order = 0;

    std::vector<TetAttributes> ta(3);
    mesh.create_mesh_attributes(va, ta);
}

void set_surface(TetWildMesh& mesh, size_t x, size_t y, size_t z, bool val)
{
    auto [tup, fid] = mesh.tuple_from_face(std::array<size_t, 3>{{x, y, z}});
    (void)tup;
    REQUIRE(fid != static_cast<size_t>(-1));
    mesh.m_face_attribute[fid].m_is_surface_fs = val;
}

bool is_surface(TetWildMesh& mesh, size_t x, size_t y, size_t z)
{
    auto [tup, fid] = mesh.tuple_from_face(std::array<size_t, 3>{{x, y, z}});
    (void)tup;
    if (fid == static_cast<size_t>(-1)) return false;
    return mesh.m_face_attribute[fid].m_is_surface_fs;
}

// A permissive envelope so the surface flip is never rejected by the Hausdorff
// check (eps large relative to the unit-size mesh).
void init_loose_envelope(SampleEnvelope& env)
{
    std::vector<Vector3d> v =
        {{0, 0, -1}, {0, 0, 1}, {1, 0, 0}, {-0.5, 0.8660254, 0}, {-0.5, -0.8660254, 0}};
    std::vector<Eigen::Vector3i> f = {{A, B, C}, {A, B, D}, {A, C, D}, {B, C, D}};
    env.init(v, f, 10.0);
}

// Force the energy gate to accept: inflate the incident tets' quality so any
// finite post-flip energy is an improvement.
void inflate_quality(TetWildMesh& mesh)
{
    for (size_t t = 0; t < 3; ++t) mesh.m_tet_attribute[t].m_quality = 1e40;
}

int count_valid_tets(TetWildMesh& mesh)
{
    int n = 0;
    for (size_t i = 0; i < mesh.tet_capacity(); ++i)
        if (mesh.tuple_from_tet(i).is_valid(mesh)) ++n;
    return n;
}

bool any_inverted(TetWildMesh& mesh)
{
    for (size_t i = 0; i < mesh.tet_capacity(); ++i) {
        const auto tt = mesh.tuple_from_tet(i);
        if (tt.is_valid(mesh) && mesh.is_inverted(tt)) return true;
    }
    return false;
}

} // namespace

TEST_CASE("surface-flip-topology-signature", "[tetwild_operation][surface_swap]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(1, 1, 1));
    SampleEnvelope env;
    init_loose_envelope(env);
    TetWildMesh mesh(params, env, 1);
    build_ring(mesh);
    set_surface(mesh, A, B, C, true);
    set_surface(mesh, A, B, D, true);

    const auto sig = mesh.surface_topology_signature();
    // Two triangles sharing edge (a,b): V=4 {a,b,c,d}, E=5 {ab,ac,bc,ad,bd},
    // F=2, one connected component, boundary loop a-c-b-d.
    CHECK(sig.F == 2);
    CHECK(sig.V == 4);
    CHECK(sig.E == 5);
    CHECK(sig.euler == 1);
    CHECK(sig.components == 1);
    CHECK(sig.boundary_loops == 1);
}

TEST_CASE("surface-flip-correctness", "[tetwild_operation][surface_swap]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(1, 1, 1));
    SampleEnvelope env;
    init_loose_envelope(env);
    TetWildMesh mesh(params, env, 1);
    build_ring(mesh);
    set_surface(mesh, A, B, C, true);
    set_surface(mesh, A, B, D, true);
    inflate_quality(mesh);

    const auto sig_before = mesh.surface_topology_signature();

    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{A, B}});
    REQUIRE(t.is_valid(mesh));
    std::vector<TetMesh::Tuple> ret;
    const bool ok = mesh.swap_edge(t, ret);

    REQUIRE(ok);
    // topology: 3 tets -> 2 tets
    CHECK(count_valid_tets(mesh) == 2);
    CHECK(mesh.check_mesh_connectivity_validity());
    CHECK_FALSE(any_inverted(mesh));

    // Surface retag: old diagonal (a,b,*) gone, new diagonal (*,c,d) surface,
    // new middle face (c,d,e) interior.
    CHECK(is_surface(mesh, A, C, D));
    CHECK(is_surface(mesh, B, C, D));
    CHECK_FALSE(is_surface(mesh, C, D, E));
    // (the old faces (a,b,c),(a,b,d) no longer exist as faces)

    // Exactly two surface faces remain.
    const auto sig_after = mesh.surface_topology_signature();
    CHECK(sig_after.F == 2);
    // Surface topology is unchanged by the flip.
    CHECK(sig_after == sig_before);

    // A 3->2 flip neither adds nor removes a surface vertex: {a,b,c,d} are on
    // the surface before and after, e stays interior. So vertex surface tags
    // (m_is_on_surface / m_order / m_is_on_open_boundary) must be invariant.
    CHECK(mesh.m_vertex_attribute[A].m_is_on_surface);
    CHECK(mesh.m_vertex_attribute[B].m_is_on_surface);
    CHECK(mesh.m_vertex_attribute[C].m_is_on_surface);
    CHECK(mesh.m_vertex_attribute[D].m_is_on_surface);
    CHECK_FALSE(mesh.m_vertex_attribute[E].m_is_on_surface);
    for (size_t v : {A, B, C, D}) CHECK(mesh.m_vertex_attribute[v].m_order == 1);
    CHECK(mesh.m_vertex_attribute[E].m_order == 0);
    for (size_t v : {A, B, C, D, E}) CHECK_FALSE(mesh.m_vertex_attribute[v].m_is_on_open_boundary);

    // The flip actually ran (counter incremented).
    CHECK(mesh.cnt_surface_swap.load() == 1);
}

TEST_CASE("surface-flip-rejected", "[tetwild_operation][surface_swap]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(1, 1, 1));

    SECTION("disabled by param -> no surface swap")
    {
        params.allow_surface_swap = false;
        SampleEnvelope env;
        init_loose_envelope(env);
        TetWildMesh mesh(params, env, 1);
        build_ring(mesh);
        set_surface(mesh, A, B, C, true);
        set_surface(mesh, A, B, D, true);
        inflate_quality(mesh);

        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{A, B}});
        std::vector<TetMesh::Tuple> ret;
        CHECK_FALSE(mesh.swap_edge(t, ret));
        // unchanged
        CHECK(is_surface(mesh, A, B, C));
        CHECK(is_surface(mesh, A, B, D));
        CHECK(count_valid_tets(mesh) == 3);
    }

    SECTION("non-manifold edge (3 surface faces) -> rejected")
    {
        SampleEnvelope env;
        init_loose_envelope(env);
        TetWildMesh mesh(params, env, 1);
        build_ring(mesh);
        set_surface(mesh, A, B, C, true);
        set_surface(mesh, A, B, D, true);
        set_surface(mesh, A, B, E, true); // 3rd surface face on edge (a,b)
        inflate_quality(mesh);

        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{A, B}});
        std::vector<TetMesh::Tuple> ret;
        CHECK_FALSE(mesh.swap_edge(t, ret));
        CHECK(count_valid_tets(mesh) == 3);
        CHECK(mesh.cnt_surface_swap.load() == 0);
    }

    SECTION("new surface face already tagged -> rejected")
    {
        SampleEnvelope env;
        init_loose_envelope(env);
        TetWildMesh mesh(params, env, 1);
        build_ring(mesh);
        set_surface(mesh, A, B, C, true);
        set_surface(mesh, A, B, D, true);
        set_surface(mesh, A, C, D, true); // one of the would-be new faces
        inflate_quality(mesh);

        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{A, B}});
        std::vector<TetMesh::Tuple> ret;
        CHECK_FALSE(mesh.swap_edge(t, ret));
        CHECK(count_valid_tets(mesh) == 3);
    }

    SECTION("energy would not improve -> rejected")
    {
        SampleEnvelope env;
        init_loose_envelope(env);
        TetWildMesh mesh(params, env, 1);
        build_ring(mesh);
        set_surface(mesh, A, B, C, true);
        set_surface(mesh, A, B, D, true);
        // Pretend the incident tets are already excellent so any flip is worse.
        for (size_t t = 0; t < 3; ++t) mesh.m_tet_attribute[t].m_quality = 1.0;

        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{A, B}});
        std::vector<TetMesh::Tuple> ret;
        CHECK_FALSE(mesh.swap_edge(t, ret));
        // rolled back
        CHECK(is_surface(mesh, A, B, C));
        CHECK(is_surface(mesh, A, B, D));
        CHECK(count_valid_tets(mesh) == 3);
    }

    SECTION("edge (c,d) already a surface edge (stale vertex flag) -> rejected")
    {
        // Six-vertex, four-tet mesh: the ring (a,b,c,d,e) plus a vertex f=5 with
        // tet T3=(b,c,d,f) sharing face (b,c,d), so edge (c,d) carries a surface
        // face (c,d,f). Flipping (a,b) would give edge (c,d) a 3rd/4th surface
        // face -> non-manifold, and must be rejected. Crucially we clear c's
        // m_is_on_surface flag (a stale state that occurs in real runs): the old
        // is_edge_on_surface()-based guard short-circuits and MISSES this; the
        // direct surface-face count on edge (c,d) still catches it.
        SampleEnvelope env;
        init_loose_envelope(env);
        TetWildMesh mesh(params, env, 1);

        constexpr size_t F = 5;
        std::vector<std::array<size_t, 4>> tets = {
            {{A, B, C, D}},
            {{A, B, D, E}},
            {{A, B, E, C}},
            {{B, C, D, F}}};
        mesh.init(6, tets);
        std::vector<VertexAttributes> va(6);
        va[A].m_posf = Vector3d(0, 0, -1);
        va[B].m_posf = Vector3d(0, 0, 1);
        va[C].m_posf = Vector3d(1, 0, 0);
        va[D].m_posf = Vector3d(-0.5, 0.8660254, 0);
        va[E].m_posf = Vector3d(-0.5, -0.8660254, 0);
        va[F].m_posf = Vector3d(0, 0, 3);
        for (int i = 0; i < 6; ++i) {
            va[i].m_is_rounded = true;
            va[i].m_pos = to_rational(va[i].m_posf);
            va[i].m_is_on_surface = true;
            va[i].m_order = 1;
        }
        va[E].m_is_on_surface = false;
        va[E].m_order = 0;
        va[C].m_is_on_surface = false; // STALE flag: c is on surface faces but flag is false
        std::vector<TetAttributes> ta(4);
        mesh.create_mesh_attributes(va, ta);

        set_surface(mesh, A, B, C, true);
        set_surface(mesh, A, B, D, true);
        set_surface(mesh, C, D, F, true); // makes (c,d) an existing surface edge
        for (size_t t = 0; t < 4; ++t) mesh.m_tet_attribute[t].m_quality = 1e40;

        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{A, B}});
        std::vector<TetMesh::Tuple> ret;
        CHECK_FALSE(mesh.swap_edge(t, ret));
        CHECK(mesh.cnt_surface_swap.load() == 0);
        CHECK(is_surface(mesh, A, B, C));
        CHECK(is_surface(mesh, A, B, D));
        CHECK(is_surface(mesh, C, D, F));
    }

    SECTION("new surface outside envelope -> rejected")
    {
        SampleEnvelope env;
        // Tight envelope around the *original* surface (a,b,c),(a,b,d). The
        // flipped faces (a,c,d),(b,c,d) deviate from it by ~0.29 (their interiors
        // leave the y=0 / (a,b,d) planes), far more than eps, so they are
        // outside and the flip must be rejected.
        std::vector<Vector3d> v =
            {{0, 0, -1}, {0, 0, 1}, {1, 0, 0}, {-0.5, 0.8660254, 0}, {-0.5, -0.8660254, 0}};
        std::vector<Eigen::Vector3i> f = {{A, B, C}, {A, B, D}};
        env.init(v, f, 1e-3);
        TetWildMesh mesh(params, env, 1);
        build_ring(mesh);
        set_surface(mesh, A, B, C, true);
        set_surface(mesh, A, B, D, true);
        inflate_quality(mesh);

        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{A, B}});
        std::vector<TetMesh::Tuple> ret;
        CHECK_FALSE(mesh.swap_edge(t, ret));
        // rolled back
        CHECK(is_surface(mesh, A, B, C));
        CHECK(is_surface(mesh, A, B, D));
        CHECK(count_valid_tets(mesh) == 3);
    }
}
