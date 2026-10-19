// The 2D counterpart of tetwild's test_input_sets.cpp: each tracked segment carries its INPUT
// SET (SurfaceTagAttributes::input_set()) and stays within eps of every input in it. A split's
// halves copy the set, the segments a collapse makes one unite theirs (2D has no surface flip),
// and the containment check holds each segment to its own set's envelopes. Below: the two
// operations on a hand-built fan, and a square crossed by an open polyline through the
// arrangement and the optimizer.

#include <wmtk/components/triwild/TriWildMesh.h>
#include <wmtk/envelope/CompositeEnvelopes.hpp>
#include <wmtk/utils/EmbedSegments.hpp>
#include <wmtk/utils/Logger.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::triwild;

namespace {

std::shared_ptr<SampleEnvelope>
envelope_around(const std::vector<Vector2d>& v, const std::vector<Vector2i>& e, double eps)
{
    auto env = std::make_shared<SampleEnvelope>();
    env->init(v, e, eps);
    return env;
}

// The fan: x at the origin, and around it on the unit circle v2 at -30 degrees, v1 at 30, a at
// 120, b at 240; triangles (x,v2,v1), (x,v1,a), (x,a,b), (x,b,v2). Collapsing v1 into v2 removes
// (x,v2,v1) and makes (x,v1) and (x,v2) one segment.
constexpr size_t X = 0, V2 = 1, V1 = 2, A = 3, B = 4;

std::vector<Vector2d> fan_points()
{
    const auto at = [](double deg) {
        const double r = deg * std::acos(-1.0) / 180;
        return Vector2d(std::cos(r), std::sin(r));
    };
    return {Vector2d(0, 0), at(-30), at(30), at(120), at(240)};
}

/// The fan with (x,v1) tracked as input set `s1` and (x,v2) as `s2`, every face's quality high
/// so no operation is refused on quality, and m_envelope loose.
void build_fan(TriWildMesh& mesh, uint16_t s1, uint16_t s2)
{
    const auto p = fan_points();
    mesh.init(
        5,
        std::vector<std::array<size_t, 3>>{{{X, V2, V1}}, {{X, V1, A}}, {{X, A, B}}, {{X, B, V2}}});
    mesh.m_vertex_attribute.resize(5);
    mesh.m_edge_attribute.resize(12);
    mesh.m_face_attribute.resize(4);
    for (size_t i = 0; i < 5; ++i) {
        auto& va = mesh.m_vertex_attribute[i];
        va.m_posf = p[i];
        va.set_pos_to_posf();
        va.m_is_rounded = true;
    }
    for (const size_t i : {X, V1, V2}) mesh.m_vertex_attribute[i].m_is_on_surface = true;
    for (size_t f = 0; f < 4; ++f) mesh.m_face_attribute[f].m_quality = 1e40;
    for (const auto& [v, s] : {std::pair<size_t, uint16_t>{V1, s1}, {V2, s2}}) {
        auto& a = mesh.m_edge_attribute[std::get<1>(mesh.tuple_from_edge({{X, v}}))];
        a.m_is_surface_fs = true;
        a.set_input_set(s);
    }
    mesh.m_envelope = envelope_around(p, {Vector2i(X, V1), Vector2i(X, V2)}, 10.0);
}

EdgeAttributes& edge(TriWildMesh& mesh, size_t a, size_t b)
{
    return mesh.m_edge_attribute[std::get<1>(mesh.tuple_from_edge({{a, b}}))];
}

bool collapse_v1_into_v2(TriWildMesh& mesh)
{
    auto t = std::get<0>(mesh.tuple_from_edge({{V1, V2}}));
    REQUIRE(t.is_valid(mesh));
    if (t.vid(mesh) != V1) t = t.switch_vertex(mesh);
    std::vector<TriMesh::Tuple> new_t;
    return mesh.collapse_edge(t, new_t);
}

std::shared_ptr<SampleEnvelope> around_fan_edge(size_t a, size_t b, double eps)
{
    const auto p = fan_points();
    return envelope_around({p[a], p[b]}, {Vector2i(0, 1)}, eps);
}

std::shared_ptr<SampleEnvelope> loose_fan_envelope()
{
    const auto p = fan_points();
    return envelope_around(p, {Vector2i(X, V1), Vector2i(X, V2)}, 10.0);
}

Parameters make_params()
{
    Parameters params;
    params.init(Vector2d(-1, -1), Vector2d(1, 1));
    return params;
}

} // namespace

TEST_CASE("input-set-2d-split-copies", "[triwild][input_sets]")
{
    Parameters params = make_params();
    TriWildMesh mesh(params, 1e-2, 0);
    build_fan(mesh, 0b01, 0b10);
    mesh.m_input_envelopes.set({around_fan_edge(X, V1, 1e-2), around_fan_edge(X, V2, 1e-2)});
    auto t = std::get<0>(mesh.tuple_from_edge({{X, V1}}));
    std::vector<TriMesh::Tuple> new_t;
    REQUIRE(mesh.split_edge(t, new_t));
    const size_t m = mesh.vert_capacity() - 1;
    CHECK(edge(mesh, X, m).m_is_surface_fs);
    CHECK(edge(mesh, X, m).input_set() == 0b01);
    CHECK(edge(mesh, m, V1).input_set() == 0b01);
    CHECK(edge(mesh, X, V2).input_set() == 0b10);
}

TEST_CASE("input-set-2d-collapse-unites", "[triwild][input_sets]")
{
    Parameters params = make_params();
    SECTION("the merged segment carries both sets")
    {
        TriWildMesh mesh(params, 1e-2, 0);
        build_fan(mesh, 0b01, 0b10);
        mesh.m_input_envelopes.set({loose_fan_envelope(), loose_fan_envelope()});
        REQUIRE(collapse_v1_into_v2(mesh));
        CHECK(edge(mesh, X, V2).m_is_surface_fs);
        CHECK(edge(mesh, X, V2).input_set() == 0b11);
    }
    SECTION("refused when the merged segment leaves one input's envelope")
    {
        // Input 0 is the segment (x, v1) itself; the merged one lies on (x, v2), 60 degrees
        // away. Union semantics would accept it: input 1 holds it.
        TriWildMesh mesh(params, 1e-2, 0);
        build_fan(mesh, 0b01, 0b10);
        mesh.m_input_envelopes.set({around_fan_edge(X, V1, 1e-2), loose_fan_envelope()});
        CHECK_FALSE(collapse_v1_into_v2(mesh));
        CHECK(edge(mesh, X, V1).input_set() == 0b01);
    }
    SECTION("the same move, accepted when both segments are input 1's")
    {
        TriWildMesh mesh(params, 1e-2, 0);
        build_fan(mesh, 0b10, 0b10);
        mesh.m_input_envelopes.set({around_fan_edge(X, V1, 1e-2), loose_fan_envelope()});
        REQUIRE(collapse_v1_into_v2(mesh));
        CHECK(edge(mesh, X, V2).input_set() == 0b10);
    }
}

TEST_CASE("input-sets-2d-square-and-crossing-polyline", "[triwild][input_sets]")
{
    // Input 0: the unit square. Input 1: an open polyline y = 0.45 + 0.1 x over [-0.5, 1.5],
    // which crosses the square's left and right sides. Through the arrangement, init_mesh and a
    // few optimization rounds, every tracked segment is inside the envelope of each input in
    // its set.
    std::vector<Vector2d> v = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    std::vector<std::array<int, 2>> e = {{{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}}};
    std::vector<int> e_input = {0, 0, 0, 0};
    const int n = 8;
    for (int i = 0; i <= n; ++i) {
        const double x = -0.5 + 2.0 * i / n;
        v.emplace_back(x, 0.45 + 0.1 * x);
        if (i > 0) {
            e.push_back({{int(v.size()) - 2, int(v.size()) - 1}});
            e_input.push_back(1);
        }
    }
    MatrixXd V(v.size(), 2);
    for (size_t i = 0; i < v.size(); ++i) V.row(i) = v[i];
    MatrixXi E(e.size(), 2);
    for (size_t i = 0; i < e.size(); ++i) E.row(i) << e[i][0], e[i][1];

    Parameters params;
    params.init(V.colwise().minCoeff(), V.colwise().maxCoeff());
    MatrixXd Vo;
    std::vector<Vector2r> Vr;
    MatrixXi F, Eo;
    std::vector<std::vector<int>> sources;
    std::vector<int> orientation;
    utils::embed_segments(V, E, Vo, Vr, F, Eo, &sources, &orientation);
    std::vector<uint16_t> e_set(Eo.rows(), 0);
    for (int i = 0; i < Eo.rows(); ++i) {
        for (const int s : sources[i]) e_set[i] |= uint16_t(1u << e_input[s]);
    }

    TriWildMesh mesh(params, params.eps, 0);
    mesh.init_mesh(Vo, Vr, F, Eo, {"input"}, V, E, &orientation, {}, &e_input, &e_set);
    REQUIRE(mesh.m_input_envelopes.size() == 2);

    const auto check_all = [&](const char* when) {
        size_t n_tracked = 0, n_outside = 0, n_empty = 0;
        std::array<size_t, 4> by_set{{0, 0, 0, 0}};
        for (const auto& t : mesh.get_edges()) {
            const EdgeAttributes& a = mesh.m_edge_attribute[t.eid(mesh)];
            if (!a.m_is_surface_fs) continue;
            ++n_tracked;
            const uint16_t s = a.input_set();
            if (s == 0 || s > 3) {
                ++n_empty;
                continue;
            }
            ++by_set[s];
            const std::array<Vector2d, 2> seg{
                {mesh.m_vertex_attribute[t.vid(mesh)].m_posf,
                 mesh.m_vertex_attribute[t.switch_vertex(mesh).vid(mesh)].m_posf}};
            for (int k = 0; k < 2; ++k) {
                if ((s & (1u << k)) && mesh.m_input_envelopes.member(k)->is_outside(seg)) {
                    ++n_outside;
                }
            }
        }
        logger().info(
            "[input sets 2d] {}: {} tracked segments, {} on the square, {} on the polyline, {} "
            "on both; {} outside an envelope of their set, {} with no valid set",
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
