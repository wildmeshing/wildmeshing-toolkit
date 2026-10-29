// TrackedRecorder<2>, the recorder of a mesh's tracked curves (TriMesh::start_curve_history), on
// the changes a mesh reports: what each becomes in the history, and what the replay of the
// history holds afterwards.

#include <wmtk/history/History.hpp>
#include <wmtk/history/Replay.hpp>
#include <wmtk/history/TrackedRecorder.hpp>

#include <catch2/catch_test_macros.hpp>

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
