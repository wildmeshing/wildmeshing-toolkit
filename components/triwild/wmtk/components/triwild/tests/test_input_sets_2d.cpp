// The 2D counterpart of tetwild's test_input_sets.cpp: each tracked segment carries its INPUT
// SET (SurfaceTagAttributes::input_set()) and stays within eps of every input in it. A split's
// halves copy the set, the segments a collapse makes one unite theirs (2D has no surface flip),
// and the containment check holds each segment to its own set's envelopes. Below: the two
// operations on a hand-built fan, and a square crossed by an open polyline through the
// arrangement and the optimizer.

#include <wmtk/components/triwild/TriWildMesh.h>
#include <wmtk/components/triwild/triwild.hpp>
#include <wmtk/envelope/CompositeEnvelopes.hpp>
#include <wmtk/history/Replay.hpp>
#include <wmtk/utils/EmbedSegments.hpp>
#include <wmtk/utils/Logger.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::triwild;

namespace {

std::shared_ptr<Envelope>
envelope_around(const std::vector<Vector2d>& v, const std::vector<Vector2i>& e, double eps)
{
    auto env = std::make_shared<Envelope>();
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
void build_fan(TriWildMesh& mesh, InputSet s1, InputSet s2)
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
    for (const auto& [v, s] : {std::pair<size_t, InputSet>{V1, s1}, {V2, s2}}) {
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

std::shared_ptr<Envelope> around_fan_edge(size_t a, size_t b, double eps)
{
    const auto p = fan_points();
    return envelope_around({p[a], p[b]}, {Vector2i(0, 1)}, eps);
}

std::shared_ptr<Envelope> loose_fan_envelope()
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
    std::vector<InputSet> e_set(Eo.rows(), 0);
    for (int i = 0; i < Eo.rows(); ++i) {
        for (const int s : sources[i]) e_set[i] |= InputSet(1) << e_input[s];
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
            const InputSet s = a.input_set();
            if (s == 0 || s > 3) {
                ++n_empty;
                continue;
            }
            ++by_set[s];
            const std::array<Vector2d, 2> seg{
                {mesh.m_vertex_attribute[t.vid(mesh)].m_posf,
                 mesh.m_vertex_attribute[t.switch_vertex(mesh).vid(mesh)].m_posf}};
            for (int k = 0; k < 2; ++k) {
                if ((s & (InputSet(1) << k)) && mesh.m_input_envelopes.member(k)->is_outside(seg)) {
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

TEST_CASE("input-sets-2d-two-inputs-nothing-simplified", "[triwild][input_sets]")
{
    // Two inputs of one segment each, crossing: the simplification removes nothing. It used to
    // report no surviving segment then, so the simplified curves lost their inputs: with the
    // optimization's envelope around them, every input set named an input with no envelope, and
    // triwild threw.
    MeshMap meshes;
    for (const auto& [name, y0] : {std::pair<std::string, double>{"a", -1}, {"b", 1}}) {
        MeshData m;
        m.V.resize(2, 3);
        m.V << -1, y0, 0, 1, -y0, 0;
        m.cells[CellType::Edge].connectivity.resize(1, 2);
        m.cells[CellType::Edge].connectivity << 0, 1;
        meshes.emplace(name, m);
    }
    for (const bool around_simplified : {false, true}) {
        INFO("optimize_envelope_around_simplified " << around_simplified);
        const ComponentResult r = components::triwild::triwild(
            {{"input", {"a", "b"}},
             {"optimize_envelope_around_simplified", around_simplified},
             {"length_rel", 0.1},
             {"max_iterations", 3}},
            meshes);
        // The crossing cuts both segments in two.
        CHECK(r.first.at("surface").cell(CellType::Edge).connectivity.rows() >= 4);
    }
}

TEST_CASE("input-sets-2d-survive-removing-triangles", "[triwild][input_sets]")
{
    // An edge's attributes live with its lowest triangle. Removing that triangle, as a filter
    // does, moves them to the other's slot: the edge stays tracked, with its set. Both tracked
    // edges of the fan have triangle 0, (x,v2,v1), as their lowest.
    Parameters params = make_params();
    TriWildMesh mesh(params, 1e-2, 0);
    build_fan(mesh, 0b01, 0b10);
    REQUIRE(std::get<1>(mesh.tuple_from_edge({{X, V1}})) / 3 == 0);
    REQUIRE(std::get<1>(mesh.tuple_from_edge({{X, V2}})) / 3 == 0);
    mesh.remove_tris_by_ids({0});
    for (const auto& [v, s] : {std::pair<size_t, InputSet>{V1, 0b01}, {V2, 0b10}}) {
        INFO("edge (x, " << v << ")");
        const EdgeAttributes& a = edge(mesh, X, v);
        CHECK(a.m_is_surface_fs);
        CHECK(a.input_set() == s);
    }
}

TEST_CASE("triwild-filter-keeps-tracked-edges", "[triwild][input_sets]")
{
    // A filter removes triangles after the optimization, so the mesh before it is the unfiltered
    // run's. Every edge of a filtered mesh that the unfiltered run tracks is tracked there too:
    // an edge whose lowest triangle went used to read that slot's last attributes, and on this
    // input lost about a quarter of the tracked edges (104 of 144 with filter input).
    const auto curves = []() {
        const int n = 300, m = 100;
        MeshData circle, wave;
        circle.V.resize(n, 3);
        auto& C = circle.cells[CellType::Edge].connectivity;
        C.resize(n, 2);
        for (int i = 0; i < n; ++i) {
            const double a = 2 * M_PI * i / n;
            circle.V.row(i) << std::cos(a), std::sin(a), 0;
            C.row(i) << i, (i + 1) % n;
        }
        wave.V.resize(m + 1, 3);
        auto& W = wave.cells[CellType::Edge].connectivity;
        W.resize(m, 2);
        for (int i = 0; i <= m; ++i) {
            const double x = -1.6 + 3.2 * i / m;
            wave.V.row(i) << x, 0.3 * std::sin(6 * x) + 0.1, 0;
            if (i > 0) W.row(i - 1) << i - 1, i;
        }
        return MeshMap{{"circle", circle}, {"wave", wave}};
    }();
    using Edge = std::array<std::array<double, 2>, 2>;
    const auto edge_of = [](const MeshData& m, int64_t a, int64_t b) {
        Edge e{{{{m.V(a, 0), m.V(a, 1)}}, {{m.V(b, 0), m.V(b, 1)}}}};
        if (e[1] < e[0]) std::swap(e[0], e[1]);
        return e;
    };
    const auto run = [&](const std::string& filter) {
        return components::triwild::triwild(
            {{"input", {"circle", "wave"}},
             {"filter", filter},
             {"length_rel", 0.02},
             {"max_iterations", 10},
             {"num_threads", 1}},
            curves);
    };
    // The edges the unfiltered run tracks, by their ends' positions.
    std::set<Edge> tracked;
    {
        const auto r = run("none");
        const MeshData& s = r.first.at("surface");
        const auto& E = s.cell(CellType::Edge).connectivity;
        for (Eigen::Index i = 0; i < E.rows(); ++i) tracked.insert(edge_of(s, E(i, 0), E(i, 1)));
    }
    REQUIRE(!tracked.empty());
    for (const std::string filter : {"input", "flood"}) {
        INFO("filter " << filter);
        const auto r = run(filter);
        std::set<Edge> expected, got;
        const MeshData& mesh = r.first.at("mesh");
        const auto& F = mesh.cell(CellType::Triangle).connectivity;
        for (Eigen::Index f = 0; f < F.rows(); ++f) {
            for (int k = 0; k < 3; ++k) {
                const Edge e = edge_of(mesh, F(f, k), F(f, (k + 1) % 3));
                if (tracked.count(e)) expected.insert(e);
            }
        }
        const MeshData& s = r.first.at("surface");
        const auto& E = s.cell(CellType::Edge).connectivity;
        for (Eigen::Index i = 0; i < E.rows(); ++i) got.insert(edge_of(s, E(i, 0), E(i, 1)));
        CHECK(!expected.empty());
        CHECK(expected.size() < tracked.size());
        CHECK(got == expected);
    }
}

TEST_CASE("curve-history-finishes-on-a-consolidated-mesh", "[triwild][tracking]")
{
    // A curve history names the output's points by the mesh's vertex ids, the output's rows only
    // once the mesh is consolidated: finishing it before refuses, after gives the history.
    Parameters params = make_params();
    TriWildMesh mesh(params, 1e-2, 0);
    build_fan(mesh, 0b01, 0b10);
    mesh.m_input_envelopes.set({loose_fan_envelope(), loose_fan_envelope()});
    mesh.record_curve_history("{}");
    REQUIRE(collapse_v1_into_v2(mesh));
    CHECK_THROWS(mesh.finish_curve_history());
    mesh.consolidate_mesh();
    const auto h = mesh.finish_curve_history();
    REQUIRE(h);
    // The two tracked edges became one, and its ends are vertex rows of the mesh.
    history::CurveReplay r(*h);
    r.apply_all(*h);
    size_t segments = 0;
    for (uint64_t f = 0; f < r.faces().size(); ++f) {
        if (!r.face_alive()[f]) continue;
        ++segments;
        for (const uint64_t v : r.faces()[f]) CHECK(h->output_vertex_ids[v] < mesh.vert_capacity());
    }
    CHECK(segments == 1);
}
