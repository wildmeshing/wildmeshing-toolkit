// TrackedRecorder<2>, the recorder of a mesh's tracked curves (TriMesh::start_curve_history,
// TetMesh::start_curve_history), on the changes a mesh reports: what each becomes in the history,
// and what the replay of the history holds afterwards.

#include <wmtk/TetMesh.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/history/Replay.hpp>
#include <wmtk/history/TrackedRecorder.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <set>
#include <vector>

using namespace wmtk;
using history::EditKind;
using history::TrackedCurveRecorder;
using Segment = TrackedCurveRecorder::Cell;
using Change = TrackedCurveRecorder::Change;

namespace {

/// Mesh vertices 0..5 on the x axis, x = their id, and 6 off it. The curves: 0-1-2, and 3-4;
/// feature points at 0, 2, 3 and 4, the ends, and at 5, which no segment holds.
struct Curves
{
    std::vector<Eigen::Vector3d> pos;
    TrackedCurveRecorder recorder;

    Curves()
        : pos(positions())
        , recorder(
              {{{0, 1}}, {{1, 2}}, {{3, 4}}},
              [this](size_t v) { return pos[v]; },
              [](const Segment& s) { return s; },
              {{{0}}, {{2}}, {{3}}, {{4}}, {{5}}})
    {}

    static std::vector<Eigen::Vector3d> positions()
    {
        std::vector<Eigen::Vector3d> p;
        for (int v = 0; v < 6; ++v) p.push_back(Eigen::Vector3d(v, 0, 0));
        p.push_back(Eigen::Vector3d(1, 1, 0));
        return p;
    }

    /// The replay of the history, at the end.
    history::CurveReplay replayed() const
    {
        history::CurveReplay r(recorder.history());
        r.apply_all(recorder.history());
        return r;
    }

    /// The feature points, by mesh vertex.
    std::vector<size_t> features() const
    {
        std::vector<size_t> out;
        for (const auto& f : recorder.features()) out.push_back(f[0]);
        return out;
    }
};

} // namespace

TEST_CASE("tracked-curves-lone-feature-point-collapse", "[history][curves]")
{
    // The feature point 5, on no segment, collapses onto curve vertex 4, which moves. No
    // segment changes, yet the history records the collapse, and the point goes with it. It
    // joins no two curves: not a weld.
    Curves c;
    const auto point = c.recorder.vertex_id(5);
    REQUIRE(point);
    c.pos[4] = Eigen::Vector3d(4.5, 0, 0);
    c.recorder.commit(Change{EditKind::collapse_edge, {5, 4}, {{{3, 4}}}, {{{3, 4}}}, {4}});
    const auto& h = c.recorder.history();
    REQUIRE(h.edits.size() == 1);
    CHECK(h.edits[0].kind == EditKind::collapse_edge);
    REQUIRE(h.edits[0].moves.size() == 1);
    CHECK(h.edits[0].moves[0].pos == c.pos[4]);
    CHECK(c.recorder.stats().collapse == 1);
    CHECK(c.recorder.stats().weld == 0);
    CHECK(c.features() == std::vector<size_t>{0, 2, 3, 4});
    CHECK_FALSE(c.recorder.vertex_id(5));
    const auto r = c.replayed();
    CHECK_FALSE(r.vertex_alive()[*point]);
    CHECK(r.vertices()[*c.recorder.vertex_id(4)] == c.pos[4]);
}

TEST_CASE("tracked-curves-survivor-off-the-curves", "[history][curves]")
{
    // A collapse onto a vertex of the mesh that is not on the curves: that vertex takes the
    // removed one's place on them, as a move, segments and feature points included.
    SECTION("a curve vertex")
    {
        Curves c;
        c.recorder.commit(
            Change{
                EditKind::collapse_edge,
                {1, 6},
                {{{0, 1}}, {{1, 2}}},
                {{{0, 6}}, {{6, 2}}},
                {6}});
        const auto& h = c.recorder.history();
        REQUIRE(h.edits.size() == 1);
        CHECK(h.edits[0].kind == EditKind::move_vertices);
        CHECK(c.recorder.stats().collapse == 1);
        CHECK(c.recorder.stats().weld == 0);
        const auto s = c.recorder.vertex_id(6);
        REQUIRE(s);
        CHECK_FALSE(c.recorder.vertex_id(1));
        const auto r = c.replayed();
        CHECK(r.vertices()[*s] == c.pos[6]);
        CHECK(r.faces_of(*s).size() == 2);
        CHECK(c.recorder.mesh_vertex_ids()[*s] == 6);
    }
    SECTION("a feature point no segment holds")
    {
        Curves c;
        c.recorder.commit(Change{EditKind::collapse_edge, {5, 6}, {}, {}, {6}});
        CHECK(c.recorder.history().edits.size() == 1);
        CHECK(c.recorder.history().edits[0].kind == EditKind::move_vertices);
        CHECK(c.recorder.stats().weld == 0);
        CHECK(c.features() == std::vector<size_t>{0, 2, 3, 4, 6});
        const auto r = c.replayed();
        CHECK(r.vertices()[*c.recorder.vertex_id(6)] == c.pos[6]);
    }
}

TEST_CASE("tracked-curves-weld", "[history][curves]")
{
    // Curve vertex 2, the end of 0-1-2, collapses onto 3, the end of 3-4, along no segment: the
    // two curves become one through 3. A weld.
    Curves c;
    c.recorder.commit(
        Change{EditKind::collapse_edge, {2, 3}, {{{1, 2}}, {{3, 4}}}, {{{1, 3}}, {{3, 4}}}, {3}});
    CHECK(c.recorder.history().edits.size() == 1);
    CHECK(c.recorder.history().edits[0].kind == EditKind::collapse_edge);
    CHECK(c.recorder.stats().collapse == 1);
    CHECK(c.recorder.stats().weld == 1);
    CHECK(c.features() == std::vector<size_t>{0, 3, 4, 5});
    const auto r = c.replayed();
    CHECK(r.faces_of(*c.recorder.vertex_id(3)).size() == 2);
}

namespace {

/**
 * A tet mesh whose tracked curves are a set of vertex pairs the test keeps by the history's
 * rules -- a split halves a tracked edge, a collapse renames the removed end's to the survivor
 * -- as an application keeps its edge tags, with positions in a journaled vertex attribute.
 */
class CurveTetMesh : public TetMesh
{
public:
    struct Position
    {
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
    };
    AttributeCollection<Position> pos;
    std::set<std::array<size_t, 2>> curves;
    std::set<size_t> points;
    /// Where the next smoothing moves its vertex.
    Eigen::Vector3d target = Eigen::Vector3d::Zero();

    CurveTetMesh() { p_vertex_attrs = &pos; }

    static std::array<size_t, 2> edge(size_t a, size_t b)
    {
        return {{std::min(a, b), std::max(a, b)}};
    }
    bool is_tracked_edge(const size_t tid, const int j) const override
    {
        const auto vs = oriented_tet_vids(tid);
        return curves.count(edge(vs[m_local_edges[j][0]], vs[m_local_edges[j][1]])) > 0;
    }
    std::vector<size_t> tracked_feature_points() const override
    {
        return {points.begin(), points.end()};
    }

    bool split_edge_before(const Tuple& t) override
    {
        m_ends = {{t.vid(*this), t.switch_vertex(*this).vid(*this)}};
        return true;
    }
    bool split_edge_after(const Tuple& t) override
    {
        const size_t n = t.vid(*this);
        pos[n].p = (pos[m_ends[0]].p + pos[m_ends[1]].p) / 2;
        if (curves.erase(edge(m_ends[0], m_ends[1]))) {
            curves.insert(edge(m_ends[0], n));
            curves.insert(edge(n, m_ends[1]));
        }
        return true;
    }
    bool collapse_edge_before(const Tuple& t) override
    {
        m_ends = {{t.vid(*this), t.switch_vertex(*this).vid(*this)}};
        return true;
    }
    bool collapse_edge_after(const Tuple& t) override
    {
        const auto [removed, kept] = m_ends;
        std::set<std::array<size_t, 2>> renamed;
        for (const auto& e : curves) {
            const size_t a = e[0] == removed ? kept : e[0], b = e[1] == removed ? kept : e[1];
            if (a != b) renamed.insert(edge(a, b));
        }
        curves = renamed;
        if (points.erase(removed)) points.insert(kept);
        return true;
    }
    bool smooth_after(const Tuple& t) override
    {
        pos[t.vid(*this)].p = target;
        return true;
    }

    /// The last state of the history, by mesh vertex: its segments, and where they are.
    std::set<std::array<size_t, 2>> replayed(const history::TrackedCurveRecorder& rec) const
    {
        history::CurveReplay r(rec.history());
        r.apply_all(rec.history());
        const auto mesh_of = rec.mesh_vertex_ids();
        std::set<std::array<size_t, 2>> out;
        for (uint64_t f = 0; f < r.faces().size(); ++f) {
            if (!r.face_alive()[f]) continue;
            const auto& s = r.faces()[f];
            for (const uint64_t v : s) CHECK(r.vertices()[v] == pos[mesh_of[v]].p);
            out.insert(edge(size_t(mesh_of[s[0]]), size_t(mesh_of[s[1]])));
        }
        return out;
    }

private:
    std::array<size_t, 2> m_ends = {{0, 0}};
};

std::set<std::array<size_t, 2>> as_set(const std::vector<std::array<size_t, 2>>& edges)
{
    return {edges.begin(), edges.end()};
}

} // namespace

TEST_CASE("tracked-curves-tet-mesh-operations", "[history][curves][TetMesh]")
{
    // TetMesh's hooks report each operation's tracked edges to the recorder at its commit point,
    // from the tets of the vertices it holds. Three tets around edge 1-2 and a fourth, 0-2-3-4,
    // on their side; the curve 1-2-0, its ends feature points. Split 1-2, smooth the new vertex,
    // filter the fourth tet away, consolidate. After each, the history ends at the tracked edges,
    // where the mesh has them. The collapses and swaps are checked on TetOptimizerMesh's feature
    // tags, over a whole optimization (tetwild's curve-history-follows-the-feature-tags): a
    // TetMesh this small has no collapse TetMesh::collapse_edge can name a return tuple for.
    CurveTetMesh m;
    m.init(6, {{{1, 2, 3, 4}}, {{1, 2, 4, 5}}, {{1, 2, 5, 3}}, {{2, 3, 4, 0}}});
    m.pos[0].p = Eigen::Vector3d(3, 0, 0);
    m.pos[1].p = Eigen::Vector3d(0, 0, 0);
    m.pos[2].p = Eigen::Vector3d(2, 0, 0);
    m.pos[3].p = Eigen::Vector3d(1, 1, 0);
    m.pos[4].p = Eigen::Vector3d(1, -0.5, 0.8);
    m.pos[5].p = Eigen::Vector3d(1, -0.5, -0.8);
    m.curves = {{{1, 2}}, {{0, 2}}};
    m.points = {0, 1};
    const auto rec = m.start_curve_history([&m](size_t v) { return m.pos.at(v).p; });
    const auto& h = rec->history();
    REQUIRE(m.tracked_edges() == std::vector<std::array<size_t, 2>>{{{0, 2}}, {{1, 2}}});
    const auto ends_at_the_mesh = [&]() {
        CHECK(m.replayed(*rec) == as_set(m.tracked_edges()));
        CHECK(m.replayed(*rec) == m.curves);
    };
    const auto features_are_the_mesh_s = [&]() {
        std::vector<size_t> features;
        for (const auto& f : rec->features()) features.push_back(f[0]);
        CHECK(features == m.tracked_feature_points());
    };

    std::vector<TetMesh::Tuple> new_tets;
    REQUIRE(m.split_edge(m.tuple_from_edge({{1, 2}}), new_tets));
    REQUIRE(h.edits.size() == 1);
    CHECK(h.edits.back().kind == history::EditKind::split_edge);
    const size_t n = 6;
    CHECK(m.curves == std::set<std::array<size_t, 2>>{{{0, 2}}, {{1, n}}, {{2, n}}});
    ends_at_the_mesh();
    features_are_the_mesh_s();

    m.target = Eigen::Vector3d(1.2, 0, 0);
    REQUIRE(m.smooth_vertex(m.tuple_from_vertex(n)));
    REQUIRE(h.edits.size() == 2);
    CHECK(h.edits.back().kind == history::EditKind::move_vertices);
    ends_at_the_mesh();

    // A filter removes the fourth tet: the curve loses segment 0-2, resynced as it is.
    m.remove_tets_by_ids({3});
    REQUIRE(h.edits.size() == 3);
    CHECK(h.edits.back().kind == history::EditKind::replace_faces);
    m.curves = {{{1, n}}, {{2, n}}};
    m.points.erase(0);
    ends_at_the_mesh();

    // The consolidation renumbers the vertices by rank, 0 gone, and the recorder with them; the
    // feature point 0 leaves with its vertex.
    m.consolidate_mesh();
    m.curves = {{{0, n - 1}}, {{1, n - 1}}};
    m.points = {0};
    ends_at_the_mesh();
    features_are_the_mesh_s();

    const auto st = rec->stats();
    CHECK(st.split == 1);
    CHECK(st.weld == 0);
    CHECK(st.replaced == 1);
    CHECK(st.unknown == 0);
}
