// Each tracked face carries its INPUT SET (SurfaceTagAttributes::input_set()) and stays within eps
// of every input in it. The operations carry the set -- a split's halves copy it, the faces a
// collapse makes one unite theirs, a surface flip gives both new faces the union of the two it
// replaces -- and the containment check holds each face to its own set's envelopes. Below: the
// three operations on hand-built meshes, each with a set that decides the outcome, and a cube
// crossed by a sheet through the insertion and the optimizer.

#include <wmtk/components/tetwild/TetWildMesh.h>
#include <wmtk/envelope/CompositeEnvelopes.hpp>
#include <wmtk/utils/Logger.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::tetwild;

namespace {

/// TetWildMesh with its refusal counters in reach, so a refusal is checked to be the
/// containment's (after_envelope) and nothing else's.
struct Mesh : TetWildMesh
{
    using TetWildMesh::CollapseReject;
    using TetWildMesh::SwapReject;
    using TetWildMesh::TetWildMesh;
    long collapse_refusals(CollapseReject r) const { return m_collapse_reject[size_t(r)].load(); }
    long swap_refusals(SwapReject r) const { return m_swap_reject[size_t(r)].load(); }
    std::string collapse_report() const
    {
        std::string out;
        for (int i = 0; i < int(CollapseReject::COUNT); ++i) {
            if (const long n = m_collapse_reject[size_t(i)].load()) {
                out += std::string(collapse_reject_name(CollapseReject(i))) + "=" +
                       std::to_string(n) + " ";
            }
        }
        return out;
    }
};

std::shared_ptr<SampleEnvelope>
envelope_around(const std::vector<Vector3d>& v, const std::vector<Eigen::Vector3i>& f, double eps)
{
    auto env = std::make_shared<SampleEnvelope>();
    env->init(v, f, eps);
    return env;
}

/// Positively oriented in WMTK's convention, swapping two vertices if needed.
std::array<size_t, 4> oriented(const std::vector<Vector3d>& p, std::array<size_t, 4> t)
{
    const double vol = (p[t[1]] - p[t[0]]).cross(p[t[2]] - p[t[0]]).dot(p[t[3]] - p[t[0]]);
    if (vol < 0) std::swap(t[2], t[3]);
    return t;
}

/// Build `tets` over `p`; `surface_verts` are flagged on the surface, every tet's quality is
/// set high so no operation is refused on quality.
void build(
    TetWildMesh& mesh,
    const std::vector<Vector3d>& p,
    const std::vector<std::array<size_t, 4>>& tets,
    const std::vector<size_t>& surface_verts)
{
    mesh.init(p.size(), tets);
    std::vector<VertexAttributes> va(p.size());
    for (size_t i = 0; i < p.size(); ++i) {
        va[i].m_posf = p[i];
        va[i].m_is_rounded = true;
        va[i].set_pos_to_posf();
    }
    for (const size_t i : surface_verts) {
        va[i].m_is_on_surface = true;
        va[i].m_order = 1;
    }
    std::vector<TetAttributes> ta(tets.size());
    mesh.create_mesh_attributes(va, ta);
    for (size_t t = 0; t < tets.size(); ++t) mesh.m_tet_attribute[t].m_quality = 1e40;
}

FaceAttributes& face(TetWildMesh& mesh, size_t x, size_t y, size_t z)
{
    const auto [tup, fid] = mesh.tuple_from_face(std::array<size_t, 3>{{x, y, z}});
    (void)tup;
    REQUIRE(fid != static_cast<size_t>(-1));
    return mesh.m_face_attribute[fid];
}

void set_tracked(TetWildMesh& mesh, size_t x, size_t y, size_t z, uint16_t set)
{
    FaceAttributes& a = face(mesh, x, y, z);
    a.m_is_surface_fs = true;
    a.set_input_set(set);
}

// The fan: three tets around the edge (l1, l2) on the x axis, their third and fourth vertices on
// the unit circle around it at x = 0.5 -- q at -90 degrees, v2 at 0, v1 at 60, p at 150 -- so
// T0 = (v1, v2, l1, l2), T1 = (v1, l1, l2, p), T2 = (v2, l1, l2, q). Collapsing v1 into v2
// removes T0 and makes (v1, l1, l2) and (v2, l1, l2) one face.
constexpr size_t L1 = 0, L2 = 1, V2 = 2, V1 = 3, P = 4, Q = 5;

std::vector<Vector3d> fan_points()
{
    const auto at = [](double deg) {
        const double r = deg * std::acos(-1.0) / 180;
        return Vector3d(0.5, std::cos(r), std::sin(r));
    };
    return {Vector3d(0, 0, 0), Vector3d(1, 0, 0), at(0), at(60), at(150), at(-90)};
}

void build_fan(TetWildMesh& mesh)
{
    const auto p = fan_points();
    build(
        mesh,
        p,
        {oriented(p, {{V1, V2, L1, L2}}),
         oriented(p, {{V1, L1, L2, P}}),
         oriented(p, {{V2, L1, L2, Q}})},
        {L1, L2, V1, V2});
}

/// An envelope of radius eps around one triangle of the fan.
std::shared_ptr<SampleEnvelope> around_fan_face(size_t a, size_t b, size_t c, double eps)
{
    const auto p = fan_points();
    return envelope_around({p[a], p[b], p[c]}, {Eigen::Vector3i(0, 1, 2)}, eps);
}

std::shared_ptr<SampleEnvelope> loose_fan_envelope()
{
    const auto p = fan_points();
    return envelope_around(p, {Eigen::Vector3i(L1, L2, V1), Eigen::Vector3i(L1, L2, V2)}, 10.0);
}

bool collapse_v1_into_v2(Mesh& mesh)
{
    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{V1, V2}});
    REQUIRE(t.is_valid(mesh));
    if (t.vid(mesh) != V1) t = t.switch_vertex(mesh);
    std::vector<TetMesh::Tuple> new_tets;
    const bool ok = mesh.collapse_edge(t, new_tets);
    logger().info("collapse refusals: {}", mesh.collapse_report());
    return ok;
}

// The ring of test_surface_swap.cpp: a = (0,0,-1), b = (0,0,1) and c, d, e an equilateral
// triangle in z = 0; tets (a,b,c,d), (a,b,d,e), (a,b,e,c). With (a,b,c) and (a,b,d) tracked, the
// 3->2 swap of (a,b) is the surface flip to (a,c,d), (b,c,d).
constexpr size_t RA = 0, RB = 1, RC = 2, RD = 3, RE = 4;

std::vector<Vector3d> ring_points()
{
    return {
        Vector3d(0, 0, -1),
        Vector3d(0, 0, 1),
        Vector3d(1, 0, 0),
        Vector3d(-0.5, 0.8660254037844386, 0),
        Vector3d(-0.5, -0.8660254037844386, 0)};
}

void build_ring(TetWildMesh& mesh)
{
    build(
        mesh,
        ring_points(),
        {{{RA, RB, RC, RD}}, {{RA, RB, RD, RE}}, {{RA, RB, RE, RC}}},
        {RA, RB, RC, RD});
}

bool flip_ab(TetWildMesh& mesh)
{
    const auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{RA, RB}});
    REQUIRE(t.is_valid(mesh));
    std::vector<TetMesh::Tuple> ret;
    return mesh.swap_edge(t, ret);
}

std::shared_ptr<SampleEnvelope> loose_ring_envelope()
{
    const auto p = ring_points();
    return envelope_around(
        p,
        {Eigen::Vector3i(RA, RB, RC),
         Eigen::Vector3i(RA, RB, RD),
         Eigen::Vector3i(RA, RC, RD),
         Eigen::Vector3i(RB, RC, RD)},
        10.0);
}

} // namespace

TEST_CASE("input-set-merge", "[tetwild][input_sets]")
{
    FaceAttributes a, b;
    a.m_is_surface_fs = true;
    a.set_input_set(0b0101);
    b.m_is_surface_fs = true;
    b.set_input_set(uint16_t(1u << 15));
    a.merge(b);
    CHECK(a.input_set() == ((1u << 15) | 0b0101));
    a.reset();
    CHECK(a.input_set() == 0);
    CHECK(sizeof(FaceAttributes) == 5);
}

TEST_CASE("input-set-split-copies", "[tetwild][input_sets]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(2, 2, 2));
    Mesh mesh(params, loose_fan_envelope(), 0);
    build_fan(mesh);
    set_tracked(mesh, V1, L1, L2, 0b01);
    set_tracked(mesh, V2, L1, L2, 0b10);
    // Each face is held to a tight envelope around itself; the halves lie in the parent's plane.
    mesh.m_input_envelopes.set(
        {around_fan_face(V1, L1, L2, 1e-2), around_fan_face(V2, L1, L2, 1e-2)});

    auto t = mesh.tuple_from_edge(std::array<size_t, 2>{{L1, L2}});
    REQUIRE(t.is_valid(mesh));
    std::vector<TetMesh::Tuple> new_tets;
    REQUIRE(mesh.split_edge(t, new_tets));
    const size_t m = mesh.vert_capacity() - 1;
    for (const size_t l : {L1, L2}) {
        CHECK(face(mesh, V1, l, m).m_is_surface_fs);
        CHECK(face(mesh, V1, l, m).input_set() == 0b01);
        CHECK(face(mesh, V2, l, m).m_is_surface_fs);
        CHECK(face(mesh, V2, l, m).input_set() == 0b10);
    }
}

TEST_CASE("input-set-collapse-unites", "[tetwild][input_sets]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(2, 2, 2));

    SECTION("the merged face carries both sets")
    {
        Mesh mesh(params, loose_fan_envelope(), 0);
        build_fan(mesh);
        set_tracked(mesh, V1, L1, L2, 0b01);
        set_tracked(mesh, V2, L1, L2, 0b10);
        mesh.m_input_envelopes.set({loose_fan_envelope(), loose_fan_envelope()});
        REQUIRE(collapse_v1_into_v2(mesh));
        CHECK(face(mesh, V2, L1, L2).m_is_surface_fs);
        CHECK(face(mesh, V2, L1, L2).input_set() == 0b11);
    }
    SECTION("refused when the merged face leaves the envelope of one of its inputs")
    {
        // Input 0 is the face (v1, l1, l2) itself; the merged face lies where (v2, l1, l2) is,
        // 60 degrees away. Union semantics would accept it: input 1 holds it.
        Mesh mesh(params, loose_fan_envelope(), 0);
        build_fan(mesh);
        set_tracked(mesh, V1, L1, L2, 0b01);
        set_tracked(mesh, V2, L1, L2, 0b10);
        mesh.m_input_envelopes.set({around_fan_face(V1, L1, L2, 1e-2), loose_fan_envelope()});
        CHECK_FALSE(collapse_v1_into_v2(mesh));
        CHECK(mesh.collapse_refusals(Mesh::CollapseReject::after_envelope) == 1);
        CHECK(face(mesh, V1, L1, L2).input_set() == 0b01);
    }
    SECTION("the same move, accepted when both faces are input 1's")
    {
        Mesh mesh(params, loose_fan_envelope(), 0);
        build_fan(mesh);
        set_tracked(mesh, V1, L1, L2, 0b10);
        set_tracked(mesh, V2, L1, L2, 0b10);
        mesh.m_input_envelopes.set({around_fan_face(V1, L1, L2, 1e-2), loose_fan_envelope()});
        REQUIRE(collapse_v1_into_v2(mesh));
        CHECK(face(mesh, V2, L1, L2).input_set() == 0b10);
    }
}

TEST_CASE("input-set-surface-flip-unites", "[tetwild][input_sets]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(1, 1, 1));
    const auto p = ring_points();

    SECTION("the new faces carry both sets")
    {
        Mesh mesh(params, loose_ring_envelope(), 1);
        build_ring(mesh);
        set_tracked(mesh, RA, RB, RC, 0b01);
        set_tracked(mesh, RA, RB, RD, 0b10);
        mesh.m_input_envelopes.set({loose_ring_envelope(), loose_ring_envelope()});
        REQUIRE(flip_ab(mesh));
        CHECK(face(mesh, RA, RC, RD).input_set() == 0b11);
        CHECK(face(mesh, RB, RC, RD).input_set() == 0b11);
    }
    SECTION("refused when the new faces leave one input's envelope")
    {
        // Input 1 is the face (a, b, d) itself: c is 0.87 off its plane.
        Mesh mesh(params, loose_ring_envelope(), 1);
        build_ring(mesh);
        set_tracked(mesh, RA, RB, RC, 0b01);
        set_tracked(mesh, RA, RB, RD, 0b10);
        mesh.m_input_envelopes.set(
            {loose_ring_envelope(),
             envelope_around({p[RA], p[RB], p[RD]}, {Eigen::Vector3i(0, 1, 2)}, 1e-2)});
        CHECK_FALSE(flip_ab(mesh));
        CHECK(mesh.swap_refusals(Mesh::SwapReject::after_envelope) == 1);
        CHECK(face(mesh, RA, RB, RC).input_set() == 0b01);
        CHECK(face(mesh, RA, RB, RD).input_set() == 0b10);
    }
}

TEST_CASE("input-sets-cube-and-crossing-sheet", "[tetwild][input_sets]")
{
    // Input 0: the unit cube. Input 1: a tilted open sheet z = 0.45 + 0.1 x over [-0.5, 1.5]^2,
    // which cuts the cube's side walls. After the insertion and a few optimization rounds, every
    // tracked face is inside the envelope of each input in its set.
    std::vector<Vector3d> v;
    std::vector<std::array<size_t, 3>> f;
    std::vector<int> f_input;
    {
        for (int i = 0; i < 8; ++i) v.emplace_back(i & 1, (i >> 1) & 1, (i >> 2) & 1);
        const int tris[12][3] = {
            {0, 2, 3},
            {0, 3, 1},
            {4, 5, 7},
            {4, 7, 6},
            {0, 1, 5},
            {0, 5, 4},
            {2, 6, 7},
            {2, 7, 3},
            {0, 4, 6},
            {0, 6, 2},
            {1, 3, 7},
            {1, 7, 5}};
        for (const auto& t : tris) {
            f.push_back({{size_t(t[0]), size_t(t[1]), size_t(t[2])}});
            f_input.push_back(0);
        }
        const int n = 8;
        const size_t base = v.size();
        for (int r = 0; r <= n; ++r) {
            for (int c = 0; c <= n; ++c) {
                const double x = -0.5 + 2.0 * c / n, y = -0.5 + 2.0 * r / n;
                v.emplace_back(x, y, 0.45 + 0.1 * x);
            }
        }
        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c) {
                const size_t a = base + r * (n + 1) + c, b = a + 1, d = a + n + 1, e = d + 1;
                f.push_back({{a, b, e}});
                f.push_back({{a, e, d}});
                f_input.push_back(1);
                f_input.push_back(1);
            }
        }
    }
    Parameters params;
    params.init(v, f);
    const auto envelope_of = [&](int k) {
        std::vector<Eigen::Vector3i> F;
        for (size_t i = 0; i < f.size(); ++i) {
            if (k < 0 || f_input[i] == k) F.emplace_back(int(f[i][0]), int(f[i][1]), int(f[i][2]));
        }
        return envelope_around(v, F, params.eps);
    };
    const auto union_env = envelope_of(-1);

    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;
    std::vector<int8_t> tet_face_orientation;
    std::vector<uint16_t> facet_input_set;
    {
        TetWildMesh mesh_insertion(params, union_env, 0);
        mesh_insertion.insertion_by_volumeremesher(
            v,
            f,
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface,
            &tet_face_orientation,
            {},
            {},
            {},
            nullptr,
            &f_input,
            &facet_input_set);
    }
    TetWildMesh mesh(params, union_env, 0);
    mesh.init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface,
        &tet_face_orientation,
        nullptr,
        nullptr,
        &facet_input_set);
    mesh.m_input_envelopes.set({envelope_of(0), envelope_of(1)});

    const auto check_all = [&](const char* when) {
        size_t n_tracked = 0, n_outside = 0, n_empty = 0;
        std::array<size_t, 4> by_set{{0, 0, 0, 0}};
        for (const auto& t : mesh.get_faces()) {
            const FaceAttributes& a = mesh.m_face_attribute[t.fid(mesh)];
            if (!a.m_is_surface_fs) continue;
            ++n_tracked;
            const uint16_t s = a.input_set();
            if (s == 0 || s > 3) {
                ++n_empty;
                continue;
            }
            ++by_set[s];
            const auto vs = mesh.get_face_vids(t);
            const std::array<Vector3d, 3> tri{
                {mesh.m_vertex_attribute[vs[0]].m_posf,
                 mesh.m_vertex_attribute[vs[1]].m_posf,
                 mesh.m_vertex_attribute[vs[2]].m_posf}};
            for (int k = 0; k < 2; ++k) {
                if ((s & (1u << k)) && mesh.m_input_envelopes.member(k)->is_outside(tri)) {
                    ++n_outside;
                }
            }
        }
        logger().info(
            "[input sets] {}: {} tracked faces, {} on the cube, {} on the sheet, {} on both; "
            "{} outside an envelope of their set, {} with no valid set",
            when,
            n_tracked,
            by_set[1],
            by_set[2],
            by_set[3],
            n_outside,
            n_empty);
        CHECK(n_tracked > 0);
        CHECK(by_set[1] > 0);
        CHECK(by_set[2] > 0);
        CHECK(n_empty == 0);
        CHECK(n_outside == 0);
    };
    check_all("after insertion");
    mesh.mesh_improvement(3);
    check_all("after optimization");
}
