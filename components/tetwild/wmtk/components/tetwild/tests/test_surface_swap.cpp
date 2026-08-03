#include <wmtk/TetMesh.h>
#include <wmtk/components/tetwild/TetWildMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
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

// ---------------------------------------------------------------------------
// Extended surface swaps: 4-4 and 5-6 edge flips.
//
// The surface change is always the 2D diagonal flip of the two incident surface
// faces (a,b,c),(a,b,d) -> (a,c,d),(b,c,d). The volumetric operation that realizes
// it depends on the ring size: 3->2 (done above), 4-4, or 5-6. These fixtures build
// an N-tet ring around edge (a=0,b=1) whose ring vertices lie on a regular N-gon in
// z=0, with a,b far apart on the z-axis so edge (a,b) is a bad sliver and the
// diagonal flip strictly lowers the AMIPS energy (mirrors tet_mesh_swap56_with_position).
// ---------------------------------------------------------------------------
namespace {

constexpr double kPi = 3.14159265358979323846;

// vids: a=0, b=1, ring i -> 2+i. Ring is CCW so the tets (a,b,r_i,r_{i+1}) are
// positively oriented. `half` is the |z| of a,b; a large value makes (a,b) a sliver.
void build_ring_n(TetWildMesh& mesh, int N, double half, double radius)
{
    std::vector<std::array<size_t, 4>> tets;
    for (int i = 0; i < N; ++i) tets.push_back({{0, 1, size_t(2 + i), size_t(2 + (i + 1) % N)}});
    mesh.init(N + 2, tets);

    std::vector<VertexAttributes> va(N + 2);
    va[0].m_posf = Vector3d(0, 0, -half);
    va[1].m_posf = Vector3d(0, 0, half);
    for (int i = 0; i < N; ++i) {
        const double ang = 2.0 * kPi * i / N;
        va[2 + i].m_posf = Vector3d(radius * std::cos(ang), radius * std::sin(ang), 0);
    }
    for (int i = 0; i < N + 2; ++i) {
        va[i].m_is_rounded = true;
        va[i].m_pos = to_rational(va[i].m_posf);
        va[i].m_is_on_surface = false;
        va[i].m_is_on_open_boundary = false;
        va[i].m_order = 0;
    }
    std::vector<TetAttributes> ta(N);
    mesh.create_mesh_attributes(va, ta);
}

void mark_surface_vertex(TetWildMesh& mesh, size_t v)
{
    mesh.m_vertex_attribute[v].m_is_on_surface = true;
    mesh.m_vertex_attribute[v].m_order = 1;
}

// Build an envelope around the current tracked-surface faces of `mesh` (eps in world
// units). `env` must outlive the swap (the mesh holds a reference).
void init_env_from_surface(SampleEnvelope& env, const TetWildMesh& mesh, double eps)
{
    std::vector<Vector3d> v(mesh.vert_capacity());
    for (size_t i = 0; i < mesh.vert_capacity(); ++i) v[i] = mesh.m_vertex_attribute[i].m_posf;
    std::vector<Eigen::Vector3i> f;
    for (size_t i = 0; i < mesh.tet_capacity(); ++i) {
        const auto tt = mesh.tuple_from_tet(i);
        if (!tt.is_valid(mesh)) continue;
        for (int j = 0; j < 4; ++j) {
            const auto ft = mesh.tuple_from_face(i, j);
            const size_t fid = ft.fid(mesh);
            if (fid != 4 * i + j) continue; // canonical: visit each face once
            if (!mesh.m_face_attribute[fid].m_is_surface_fs) continue;
            const auto vs = mesh.get_face_vids(ft);
            f.emplace_back(int(vs[0]), int(vs[1]), int(vs[2]));
        }
    }
    env.init(v, f, eps);
}

int count_valid_tets_n(TetWildMesh& mesh)
{
    int n = 0;
    for (size_t i = 0; i < mesh.tet_capacity(); ++i)
        if (mesh.tuple_from_tet(i).is_valid(mesh)) ++n;
    return n;
}

bool any_inverted_n(TetWildMesh& mesh)
{
    for (size_t i = 0; i < mesh.tet_capacity(); ++i) {
        const auto tt = mesh.tuple_from_tet(i);
        if (tt.is_valid(mesh) && mesh.is_inverted(tt)) return true;
    }
    return false;
}

} // namespace

TEST_CASE("surface-flip-44-correctness", "[tetwild_operation][surface_swap]")
{
    // 4-tet ring; the two surface apexes are OPPOSITE in the ring (vids 2 and 4), so
    // exactly one 4-4 diagonal is (2,4) and the flip is realizable.
    Parameters params;
    params.init(Vector3d(-2, -2, -2000), Vector3d(2, 2, 2000));
    SampleEnvelope env;
    TetWildMesh mesh(params, env, 1);
    build_ring_n(mesh, 4, 1000.0, 1.0);
    for (size_t v : {size_t(0), size_t(1), size_t(2), size_t(4)}) mark_surface_vertex(mesh, v);
    set_surface(mesh, 0, 1, 2, true); // (a,b,c)
    set_surface(mesh, 0, 1, 4, true); // (a,b,d)
    init_env_from_surface(env, mesh, 50.0); // permissive (the surface stays planar here)

    const auto sig_before = mesh.surface_topology_signature();
    CHECK(sig_before.F == 2);

    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
    REQUIRE(t.is_valid(mesh));
    std::vector<TetMesh::Tuple> ret;
    const bool ok = mesh.swap_edge_44(t, ret);

    REQUIRE(ok);
    CHECK(count_valid_tets_n(mesh) == 4); // 4-4: tet count unchanged
    CHECK(mesh.check_mesh_connectivity_validity());
    CHECK_FALSE(any_inverted_n(mesh));

    // Surface diagonal flipped (a,b) -> (c,d) == (2,4).
    CHECK(is_surface(mesh, 0, 2, 4)); // (a,c,d)
    CHECK(is_surface(mesh, 1, 2, 4)); // (b,c,d)

    const auto sig_after = mesh.surface_topology_signature();
    CHECK(sig_after.F == 2);
    CHECK(sig_after == sig_before); // surface topology invariant

    CHECK(mesh.cnt_surface_swap_44.load() == 1);
    CHECK(mesh.cnt_surface_swap.load() == 1);
    // Vertex surface membership is invariant under a pure diagonal flip.
    for (size_t v : {size_t(0), size_t(1), size_t(2), size_t(4)})
        CHECK(mesh.m_vertex_attribute[v].m_is_on_surface);
}

TEST_CASE("surface-flip-44-rejected", "[tetwild_operation][surface_swap]")
{
    Parameters params;
    params.init(Vector3d(-2, -2, -2000), Vector3d(2, 2, 2000));

    SECTION("adjacent surface apexes -> no matching diagonal -> rejected")
    {
        // Surface apexes vids 2 and 3 are ADJACENT in the ring, so neither 4-4
        // diagonal ((2,4) or (3,5)) equals (2,3): the accept-case vetoes both cases.
        SampleEnvelope env;
        TetWildMesh mesh(params, env, 1);
        build_ring_n(mesh, 4, 1000.0, 1.0);
        for (size_t v : {size_t(0), size_t(1), size_t(2), size_t(3)}) mark_surface_vertex(mesh, v);
        set_surface(mesh, 0, 1, 2, true);
        set_surface(mesh, 0, 1, 3, true);
        init_env_from_surface(env, mesh, 50.0);

        const auto sig_before = mesh.surface_topology_signature();
        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
        std::vector<TetMesh::Tuple> ret;
        CHECK_FALSE(mesh.swap_edge_44(t, ret));
        CHECK(count_valid_tets_n(mesh) == 4);
        CHECK(mesh.cnt_surface_swap_44.load() == 0);
        CHECK(mesh.surface_topology_signature() == sig_before);
    }

    SECTION("disabled by param -> rejected")
    {
        params.allow_surface_swap = false;
        SampleEnvelope env;
        TetWildMesh mesh(params, env, 1);
        build_ring_n(mesh, 4, 1000.0, 1.0);
        for (size_t v : {size_t(0), size_t(1), size_t(2), size_t(4)}) mark_surface_vertex(mesh, v);
        set_surface(mesh, 0, 1, 2, true);
        set_surface(mesh, 0, 1, 4, true);
        init_env_from_surface(env, mesh, 50.0);

        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
        std::vector<TetMesh::Tuple> ret;
        CHECK_FALSE(mesh.swap_edge_44(t, ret));
        CHECK(count_valid_tets_n(mesh) == 4);
        CHECK(mesh.cnt_surface_swap_44.load() == 0);
    }

    SECTION("non-manifold edge (3 surface faces) -> rejected")
    {
        SampleEnvelope env;
        TetWildMesh mesh(params, env, 1);
        build_ring_n(mesh, 4, 1000.0, 1.0);
        for (size_t v : {size_t(0), size_t(1), size_t(2), size_t(3), size_t(4)})
            mark_surface_vertex(mesh, v);
        set_surface(mesh, 0, 1, 2, true);
        set_surface(mesh, 0, 1, 3, true);
        set_surface(mesh, 0, 1, 4, true); // 3rd surface face on edge (a,b)
        init_env_from_surface(env, mesh, 50.0);

        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
        std::vector<TetMesh::Tuple> ret;
        CHECK_FALSE(mesh.swap_edge_44(t, ret));
        CHECK(count_valid_tets_n(mesh) == 4);
        CHECK(mesh.cnt_surface_swap_44.load() == 0);
    }
}

TEST_CASE("surface-flip-44-routing-stale-flag", "[tetwild_operation][surface_swap]")
{
    // A genuine surface edge whose endpoint vertex flags are stale-false must still be
    // routed through the surface flip (edge_incident_surface_face_count is face-based),
    // so the surface is not silently torn by the interior path.
    Parameters params;
    params.init(Vector3d(-2, -2, -2000), Vector3d(2, 2, 2000));
    SampleEnvelope env;
    TetWildMesh mesh(params, env, 1);
    build_ring_n(mesh, 4, 1000.0, 1.0);
    // Deliberately DO NOT mark any vertex on-surface (all m_is_on_surface stay false),
    // even though (0,1,2),(0,1,4) are surface faces.
    set_surface(mesh, 0, 1, 2, true);
    set_surface(mesh, 0, 1, 4, true);
    init_env_from_surface(env, mesh, 50.0);

    const auto sig_before = mesh.surface_topology_signature();
    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
    std::vector<TetMesh::Tuple> ret;
    const bool ok = mesh.swap_edge_44(t, ret);
    REQUIRE(ok);
    // The flip ran through the surface path: surface retagged and topology preserved.
    CHECK(is_surface(mesh, 0, 2, 4));
    CHECK(is_surface(mesh, 1, 2, 4));
    CHECK(mesh.cnt_surface_swap_44.load() == 1);
    CHECK(mesh.surface_topology_signature() == sig_before);
}

TEST_CASE("surface-flip-56-correctness", "[tetwild_operation][surface_swap]")
{
    // 5-tet ring; the two surface apexes (vids 2 and 4) are separated by ONE ring vertex
    // (distance 2 in the pentagon), so a fan from vertex 2 (or 4) creates the diagonal
    // (2,4) and realizes the surface flip.
    Parameters params;
    params.init(Vector3d(-2, -2, -2000), Vector3d(2, 2, 2000));
    SampleEnvelope env;
    TetWildMesh mesh(params, env, 1);
    build_ring_n(mesh, 5, 1000.0, 1.0);
    for (size_t v : {size_t(0), size_t(1), size_t(2), size_t(4)}) mark_surface_vertex(mesh, v);
    set_surface(mesh, 0, 1, 2, true); // (a,b,c)
    set_surface(mesh, 0, 1, 4, true); // (a,b,d)
    init_env_from_surface(env, mesh, 50.0); // permissive: the surface is folded here

    const auto sig_before = mesh.surface_topology_signature();
    CHECK(sig_before.F == 2);

    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
    REQUIRE(t.is_valid(mesh));
    std::vector<TetMesh::Tuple> ret;
    const bool ok = mesh.swap_edge_56(t, ret);

    REQUIRE(ok);
    CHECK(count_valid_tets_n(mesh) == 6); // 5-6: 5 tets -> 6 tets
    CHECK(mesh.check_mesh_connectivity_validity());
    CHECK_FALSE(any_inverted_n(mesh));

    CHECK(is_surface(mesh, 0, 2, 4)); // (a,c,d)
    CHECK(is_surface(mesh, 1, 2, 4)); // (b,c,d)

    const auto sig_after = mesh.surface_topology_signature();
    CHECK(sig_after.F == 2);
    CHECK(sig_after == sig_before);

    CHECK(mesh.cnt_surface_swap_56.load() == 1);
    CHECK(mesh.cnt_surface_swap.load() == 1);
}

TEST_CASE("surface-flip-56-rejected", "[tetwild_operation][surface_swap]")
{
    // Surface apexes vids 2,3 are ADJACENT. For the fan at vertex 5 the new face is
    // (5,2,3) whose edge (2,3) is a pre-existing link edge -- a naive "face contains
    // (c,d)" test would wrongly accept it; the apex-keyed accept-case (apex 5 not in
    // {2,3}) correctly rejects, so no flip happens.
    // (Envelope-based rejection is identical code to the 3->2 path and is covered by the
    // "new surface outside envelope" section of surface-flip-rejected.)
    Parameters params;
    params.init(Vector3d(-2, -2, -2000), Vector3d(2, 2, 2000));
    SampleEnvelope env;
    TetWildMesh mesh(params, env, 1);
    build_ring_n(mesh, 5, 1000.0, 1.0);
    for (size_t v : {size_t(0), size_t(1), size_t(2), size_t(3)}) mark_surface_vertex(mesh, v);
    set_surface(mesh, 0, 1, 2, true);
    set_surface(mesh, 0, 1, 3, true);
    init_env_from_surface(env, mesh, 50.0);

    const auto sig_before = mesh.surface_topology_signature();
    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
    std::vector<TetMesh::Tuple> ret;
    CHECK_FALSE(mesh.swap_edge_56(t, ret));
    CHECK(count_valid_tets_n(mesh) == 5);
    CHECK(mesh.cnt_surface_swap_56.load() == 0);
    CHECK(mesh.surface_topology_signature() == sig_before);
}

TEST_CASE("surface-flip-interior-unchanged", "[tetwild_operation][surface_swap]")
{
    // With no surface faces, the 4-4 and 5-6 swaps must behave exactly as before: the
    // accept-case predicate defaults to accepting every case (is_surface_flip == false),
    // and no surface retag occurs.
    Parameters params;
    params.init(Vector3d(-2, -2, -2000), Vector3d(2, 2, 2000));

    SECTION("interior 4-4")
    {
        SampleEnvelope env;
        TetWildMesh mesh(params, env, 1);
        build_ring_n(mesh, 4, 1000.0, 1.0); // no surface faces tagged
        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
        std::vector<TetMesh::Tuple> ret;
        CHECK(mesh.swap_edge_44(t, ret));
        CHECK(count_valid_tets_n(mesh) == 4);
        CHECK(mesh.check_mesh_connectivity_validity());
        CHECK(mesh.cnt_surface_swap_44.load() == 0);
    }

    SECTION("interior 5-6")
    {
        SampleEnvelope env;
        TetWildMesh mesh(params, env, 1);
        build_ring_n(mesh, 5, 1000.0, 1.0);
        auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
        std::vector<TetMesh::Tuple> ret;
        CHECK(mesh.swap_edge_56(t, ret));
        CHECK(count_valid_tets_n(mesh) == 6);
        CHECK(mesh.check_mesh_connectivity_validity());
        CHECK(mesh.cnt_surface_swap_56.load() == 0);
    }
}

TEST_CASE("surface-face-swap-excluded", "[tetwild_operation][surface_swap]")
{
    // The 2->3 face swap must never touch a surface face (it would tear a hole in the
    // tracked surface). Confirm it is rejected and the surface is unchanged.
    Parameters params;
    params.init(Vector3d(-2, -2, -2), Vector3d(2, 2, 2));
    SampleEnvelope env;
    init_loose_envelope(env);
    TetWildMesh mesh(params, env, 1);
    build_ring(mesh);
    set_surface(mesh, A, B, C, true);
    set_surface(mesh, A, B, D, true);

    const auto sig_before = mesh.surface_topology_signature();
    auto [ftup, fid] = mesh.tuple_from_face(std::array<size_t, 3>{{A, B, C}});
    (void)fid;
    REQUIRE(ftup.is_valid(mesh));
    std::vector<TetMesh::Tuple> ret;
    CHECK_FALSE(mesh.swap_face(ftup, ret)); // surface face: rejected
    CHECK(is_surface(mesh, A, B, C));
    CHECK(mesh.surface_topology_signature() == sig_before);
}
