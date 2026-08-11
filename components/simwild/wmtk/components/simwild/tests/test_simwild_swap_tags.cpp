#include <catch2/catch_test_macros.hpp>

#include <wmtk/TetMesh.h>
#include <wmtk/components/simwild/SimWildMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <array>
#include <cmath>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::simwild;

// ---------------------------------------------------------------------------
// simwild's surface is the interface between differently tagged tets: a face is
// tagged m_is_surface_fs exactly when its two tets carry different tags (see
// SimWildMesh::init_surfaces_and_boundaries). A surface diagonal flip is therefore a
// flip of that interface, and it moves the tet (a,b,c,d) worth of volume from one tag to
// the other. These tests check that every tet the flip creates ends up with the tag of
// the side it landed on -- equivalently, that the "surface iff tags differ" invariant
// still holds afterwards.
//
// Fixture: an N-tet ring around edge (a=0,b=1), ring vertices 2..N+1 on a regular N-gon
// in z=0 and a,b far apart on the z axis so (a,b) is a sliver and the flip strictly
// lowers the AMIPS energy. Mirrors tetwild's test_surface_swap.cpp, whose swap code this
// now shares.
// ---------------------------------------------------------------------------
namespace {

constexpr double kPi = 3.14159265358979323846;

const CellTag TAG_A{1};
const CellTag TAG_B{2};

void build_ring_n(SimWildMesh& mesh, int N, double half, double radius)
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
        va[i].m_order = 0;
    }
    std::vector<TetAttributes> ta(N);
    mesh.create_mesh_attributes(va, ta);
}

size_t fid_of(SimWildMesh& mesh, size_t x, size_t y, size_t z)
{
    auto [tup, fid] = mesh.tuple_from_face(std::array<size_t, 3>{{x, y, z}});
    (void)tup;
    return fid;
}

bool is_surface(SimWildMesh& mesh, size_t x, size_t y, size_t z)
{
    const size_t fid = fid_of(mesh, x, y, z);
    if (fid == static_cast<size_t>(-1)) return false;
    return mesh.m_face_attribute[fid].m_is_surface_fs;
}

/// Tag the ring tets: tets whose two ring vertices are both in [lo,hi] (the arc from
/// surface apex `lo` to surface apex `hi`) get TAG_A, the rest TAG_B. Then derive the
/// surface faces from the tags, exactly as init_surfaces_and_boundaries does.
void tag_arcs_and_derive_surface(SimWildMesh& mesh, int N, int lo, int hi)
{
    for (int i = 0; i < N; ++i) {
        const bool in_arc = (i >= lo && i < hi);
        mesh.m_tet_attribute[i].tags = in_arc ? TAG_A : TAG_B;
    }
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < 4; ++j) {
            const auto ft = mesh.tuple_from_face(size_t(i), j);
            const size_t fid = ft.fid(mesh);
            const auto opp = ft.switch_tetrahedron(mesh);
            const bool surf = opp.has_value() && mesh.m_tet_attribute[i].tags !=
                                                     mesh.m_tet_attribute[opp->tid(mesh)].tags;
            mesh.m_face_attribute[fid].m_is_surface_fs = surf;
            if (!surf) continue;
            for (const size_t v : mesh.get_face_vids(ft)) {
                mesh.m_vertex_attribute[v].m_is_on_surface = true;
                mesh.m_vertex_attribute[v].m_order = 1;
            }
        }
    }
}

/// The invariant the whole design rests on: a face is tagged surface exactly when it
/// separates differently tagged tets. Boundary faces (no opposite tet) are exempt --
/// init_surfaces_and_boundaries skips them.
void check_surface_matches_tags(SimWildMesh& mesh)
{
    for (size_t i = 0; i < mesh.tet_capacity(); ++i) {
        if (!mesh.tuple_from_tet(i).is_valid(mesh)) continue;
        for (int j = 0; j < 4; ++j) {
            const auto ft = mesh.tuple_from_face(i, j);
            const size_t fid = ft.fid(mesh);
            const auto opp = ft.switch_tetrahedron(mesh);
            if (!opp.has_value()) continue;
            const bool differ =
                mesh.m_tet_attribute[i].tags != mesh.m_tet_attribute[opp->tid(mesh)].tags;
            INFO("face " << fid << " of tet " << i);
            CHECK(mesh.m_face_attribute[fid].m_is_surface_fs == differ);
        }
    }
}

/// Every valid tet must carry one of the two tags -- never an empty or mixed one.
void check_tags_are_a_or_b(SimWildMesh& mesh)
{
    for (size_t i = 0; i < mesh.tet_capacity(); ++i) {
        if (!mesh.tuple_from_tet(i).is_valid(mesh)) continue;
        const CellTag& tags = mesh.m_tet_attribute[i].tags;
        INFO("tet " << i);
        CHECK((tags == TAG_A || tags == TAG_B));
    }
}

/// Tets carrying `tag`, by vids.
std::vector<std::array<size_t, 4>> tets_with_tag(SimWildMesh& mesh, const CellTag& tag)
{
    std::vector<std::array<size_t, 4>> out;
    for (size_t i = 0; i < mesh.tet_capacity(); ++i) {
        if (!mesh.tuple_from_tet(i).is_valid(mesh)) continue;
        if (mesh.m_tet_attribute[i].tags != tag) continue;
        auto vids = mesh.oriented_tet_vids(i);
        std::sort(vids.begin(), vids.end());
        out.push_back(vids);
    }
    std::sort(out.begin(), out.end());
    return out;
}

void init_env_from_surface(SampleEnvelope& env, SimWildMesh& mesh, double eps)
{
    std::vector<Vector3d> v(mesh.vert_capacity());
    for (size_t i = 0; i < mesh.vert_capacity(); ++i) v[i] = mesh.m_vertex_attribute[i].m_posf;
    std::vector<Eigen::Vector3i> f;
    for (size_t i = 0; i < mesh.tet_capacity(); ++i) {
        if (!mesh.tuple_from_tet(i).is_valid(mesh)) continue;
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

int count_valid_tets(SimWildMesh& mesh)
{
    int n = 0;
    for (size_t i = 0; i < mesh.tet_capacity(); ++i)
        if (mesh.tuple_from_tet(i).is_valid(mesh)) ++n;
    return n;
}

bool any_inverted(SimWildMesh& mesh)
{
    for (size_t i = 0; i < mesh.tet_capacity(); ++i) {
        const auto tt = mesh.tuple_from_tet(i);
        if (tt.is_valid(mesh) && mesh.is_inverted(tt)) return true;
    }
    return false;
}

void inflate_quality(SimWildMesh& mesh, int N)
{
    for (int i = 0; i < N; ++i) mesh.m_tet_attribute[i].m_quality = 1e40;
}

} // namespace

TEST_CASE("simwild-swap-tags-32", "[simwild][3D][swap][tags]")
{
    // 3-tet ring. Surface apexes c=2, d=3 are adjacent, so the arc [2,3) holds the single
    // tet (a,b,2,3) -- tag A -- and the other two are tag B. The flip absorbs the A tet:
    // both resulting tets contain ring vertex 4, which is on the B side.
    Parameters params;
    params.init(Vector3d(-2, -2, -2000), Vector3d(2, 2, 2000));
    SimWildMesh mesh(params, 1e-3, 1);
    build_ring_n(mesh, 3, 1000.0, 1.0);
    tag_arcs_and_derive_surface(mesh, 3, 0, 1);
    REQUIRE(is_surface(mesh, 0, 1, 2));
    REQUIRE(is_surface(mesh, 0, 1, 3));
    REQUIRE_FALSE(is_surface(mesh, 0, 1, 4));
    check_surface_matches_tags(mesh);

    auto env = std::make_shared<SampleEnvelope>();
    init_env_from_surface(*env, mesh, 50.0);
    mesh.m_envelope = env;
    inflate_quality(mesh, 3);

    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
    REQUIRE(t.is_valid(mesh));
    std::vector<TetMesh::Tuple> ret;
    REQUIRE(mesh.swap_edge(t, ret));

    CHECK(count_valid_tets(mesh) == 2);
    CHECK(mesh.check_mesh_connectivity_validity());
    CHECK_FALSE(any_inverted(mesh));
    CHECK(mesh.cnt_surface_swap_32.load() == 1);

    // Both new tets are on the B side of the flipped interface.
    CHECK(tets_with_tag(mesh, TAG_A).empty());
    CHECK(tets_with_tag(mesh, TAG_B).size() == 2);
    check_tags_are_a_or_b(mesh);
    check_surface_matches_tags(mesh);
}

TEST_CASE("simwild-swap-tags-44", "[simwild][3D][swap][tags]")
{
    // 4-tet ring with the surface apexes OPPOSITE (c=2, d=4), the only configuration a 4-4
    // diagonal can realize. Arc [2,4) = tets 0,1 (the ones holding ring vertex 3) is tag A,
    // tets 2,3 (holding ring vertex 5) are tag B. A 4-4 flip is volume-neutral in tet count,
    // so both tags must survive with two tets each.
    Parameters params;
    params.init(Vector3d(-2, -2, -2000), Vector3d(2, 2, 2000));
    SimWildMesh mesh(params, 1e-3, 1);
    build_ring_n(mesh, 4, 1000.0, 1.0);
    tag_arcs_and_derive_surface(mesh, 4, 0, 2);
    REQUIRE(is_surface(mesh, 0, 1, 2));
    REQUIRE(is_surface(mesh, 0, 1, 4));
    check_surface_matches_tags(mesh);

    auto env = std::make_shared<SampleEnvelope>();
    init_env_from_surface(*env, mesh, 50.0);
    mesh.m_envelope = env;
    inflate_quality(mesh, 4);

    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
    REQUIRE(t.is_valid(mesh));
    std::vector<TetMesh::Tuple> ret;
    REQUIRE(mesh.swap_edge_44(t, ret));

    CHECK(count_valid_tets(mesh) == 4);
    CHECK(mesh.check_mesh_connectivity_validity());
    CHECK_FALSE(any_inverted(mesh));
    CHECK(mesh.cnt_surface_swap_44.load() == 1);

    // The new interface is (0,2,4),(1,2,4); the A side keeps ring vertex 3, the B side 5.
    CHECK(is_surface(mesh, 0, 2, 4));
    CHECK(is_surface(mesh, 1, 2, 4));
    CHECK(
        tets_with_tag(mesh, TAG_A) ==
        std::vector<std::array<size_t, 4>>{{{0, 2, 3, 4}}, {{1, 2, 3, 4}}});
    CHECK(
        tets_with_tag(mesh, TAG_B) ==
        std::vector<std::array<size_t, 4>>{{{0, 2, 4, 5}}, {{1, 2, 4, 5}}});
    check_tags_are_a_or_b(mesh);
    check_surface_matches_tags(mesh);
}

TEST_CASE("simwild-swap-tags-56", "[simwild][3D][swap][tags]")
{
    // 5-tet ring, surface apexes c=2, d=4. Arc [2,4) = tets 0,1 (ring vertices 2,3,4) is
    // tag A; tets 2,3,4 (ring vertices 4,5,6,2) are tag B.
    Parameters params;
    params.init(Vector3d(-2, -2, -2000), Vector3d(2, 2, 2000));
    SimWildMesh mesh(params, 1e-3, 1);
    build_ring_n(mesh, 5, 1000.0, 1.0);
    tag_arcs_and_derive_surface(mesh, 5, 0, 2);
    REQUIRE(is_surface(mesh, 0, 1, 2));
    REQUIRE(is_surface(mesh, 0, 1, 4));
    check_surface_matches_tags(mesh);

    auto env = std::make_shared<SampleEnvelope>();
    init_env_from_surface(*env, mesh, 50.0);
    mesh.m_envelope = env;
    inflate_quality(mesh, 5);

    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
    REQUIRE(t.is_valid(mesh));
    std::vector<TetMesh::Tuple> ret;
    REQUIRE(mesh.swap_edge_56(t, ret));

    CHECK(count_valid_tets(mesh) == 6);
    CHECK(mesh.check_mesh_connectivity_validity());
    CHECK_FALSE(any_inverted(mesh));
    CHECK(mesh.cnt_surface_swap_56.load() == 1);

    CHECK(is_surface(mesh, 0, 2, 4));
    CHECK(is_surface(mesh, 1, 2, 4));
    // The fan is over the pentagon from apex 2 or 4; either way the A side is the pair of
    // tets holding ring vertex 3 and the B side is the remaining four.
    CHECK(tets_with_tag(mesh, TAG_A).size() == 2);
    CHECK(tets_with_tag(mesh, TAG_B).size() == 4);
    check_tags_are_a_or_b(mesh);
    check_surface_matches_tags(mesh);
}

TEST_CASE("simwild-swap-tags-interior", "[simwild][3D][swap][tags]")
{
    // No surface face anywhere: every tet carries the same tag and every tet the swap
    // creates must keep it. This is the path that used to read tags off incident_tets[0]
    // (4-4 / 5-6) or majority-vote them (3->2).
    Parameters params;
    params.init(Vector3d(-2, -2, -2000), Vector3d(2, 2, 2000));
    SimWildMesh mesh(params, 1e-3, 1);
    build_ring_n(mesh, 4, 1000.0, 1.0);
    tag_arcs_and_derive_surface(mesh, 4, 0, 4); // everything tag A -> no interface at all
    for (size_t i = 0; i < mesh.tet_capacity(); ++i) {
        if (!mesh.tuple_from_tet(i).is_valid(mesh)) continue;
        for (int j = 0; j < 4; ++j) {
            REQUIRE_FALSE(
                mesh.m_face_attribute[mesh.tuple_from_face(i, j).fid(mesh)].m_is_surface_fs);
        }
    }

    // No envelope on purpose: there is no surface here, and the envelope is only consulted
    // on the surface-flip path. (Handing SampleEnvelope an empty face set recurses forever.)
    inflate_quality(mesh, 4);

    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{0, 1}});
    std::vector<TetMesh::Tuple> ret;
    REQUIRE(mesh.swap_edge_44(t, ret));

    CHECK(mesh.cnt_surface_swap.load() == 0); // interior swap, not a surface flip
    CHECK(tets_with_tag(mesh, TAG_A).size() == 4);
    CHECK(tets_with_tag(mesh, TAG_B).empty());
    check_surface_matches_tags(mesh);
}
