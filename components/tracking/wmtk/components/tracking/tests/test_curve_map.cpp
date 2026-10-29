#include <wmtk/components/tracking/CurveMap.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/history/Replay.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <random>
#include <thread>

using namespace wmtk;
using namespace wmtk::components;
using history::CurveHistory;
using history::Edit;
using history::EditKind;
using history::kNoVertex;
using tracking::CurveMap;
using tracking::CurvePoint;

namespace {

/// A polyline along the x axis: vertices at `xs`, a segment between each two in a row.
CurveHistory polyline(const std::vector<double>& xs)
{
    CurveHistory h;
    for (const double x : xs) h.vertices.push_back(Eigen::Vector3d(x, 0, 0));
    h.vertex_alive.assign(xs.size(), 1);
    for (uint64_t i = 0; i + 1 < xs.size(); ++i) h.faces.push_back({{i, i + 1}});
    h.face_alive.assign(h.faces.size(), 1);
    return h;
}

Eigen::Vector3d at(
    const CurvePoint& p,
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<std::array<uint64_t, 2>>& S)
{
    return p.bary[0] * V[S[p.segment][0]] + p.bary[1] * V[S[p.segment][1]];
}

/// A number in [0, 1) from the generator alone: mt19937 gives the same numbers everywhere, but
/// std::uniform_real_distribution does not -- libstdc++ and libc++ draw them differently -- and
/// the points mapped here must not depend on the standard library.
double uniform(std::mt19937& rng)
{
    return double(rng()) / 4294967296.0;
}

std::vector<CurvePoint> random_points(const CurveMap& map, const bool on_input, const int n)
{
    const auto& alive = on_input ? map.input_segment_alive() : map.output_segment_alive();
    std::vector<uint64_t> live;
    for (uint64_t f = 0; f < alive.size(); ++f) {
        if (alive[f]) live.push_back(f);
    }
    std::mt19937 rng(7);
    std::vector<CurvePoint> pts;
    for (int i = 0; i < n; ++i) {
        const double t = uniform(rng);
        pts.push_back({live[rng() % live.size()], Eigen::Vector2d(1 - t, t)});
    }
    return pts;
}

/// Every point lands where it was, and comes back where it started.
void check_identity_in_space(const CurveMap& map)
{
    for (const bool on_input : {true, false}) {
        const auto pts = random_points(map, on_input, 300);
        const auto there = on_input ? map.to_output(pts) : map.to_input(pts);
        const auto back = on_input ? map.to_input(there) : map.to_output(there);
        const auto& V0 = on_input ? map.input_vertices() : map.output_vertices();
        const auto& S0 = on_input ? map.input_segments() : map.output_segments();
        const auto& V1 = on_input ? map.output_vertices() : map.input_vertices();
        const auto& S1 = on_input ? map.output_segments() : map.input_segments();
        for (size_t i = 0; i < pts.size(); ++i) {
            REQUIRE(there[i].segment != kNoVertex);
            CHECK((at(there[i], V1, S1) - at(pts[i], V0, S0)).norm() < 1e-12);
            REQUIRE(back[i].segment == pts[i].segment);
            CHECK((back[i].bary - pts[i].bary).norm() < 1e-12);
        }
    }
}

} // namespace

TEST_CASE("curve-map-straight-line", "[tracking][curves]")
{
    // A straight polyline whose ends stay put: by arc length, every operation along it is the
    // identity in space.
    CurveHistory h = polyline({0, 1, 2, 3, 4});
    h.features = {{{0}}, {{4}}};
    // Split segment 1 = (1,2) at 1.25 (not its midpoint), as vertex 5 with segment 4.
    h.edits.push_back(
        {EditKind::split_edge, {1, 2, 5}, {1, 4}, {{5, Eigen::Vector3d(1.25, 0, 0)}}});
    // Collapse vertex 2 into 3, which moves to 2.5.
    h.edits.push_back({EditKind::collapse_edge, {2, 3}, {}, {{3, Eigen::Vector3d(2.5, 0, 0)}}});
    // Slide vertex 1 along the line, then vertex 5.
    h.edits.push_back({EditKind::move_vertices, {}, {}, {{1, Eigen::Vector3d(0.7, 0, 0)}}});
    h.edits.push_back({EditKind::move_vertices, {}, {}, {{5, Eigen::Vector3d(1.9, 0, 0)}}});
    h.edits.push_back({EditKind::consolidate, {}, {}, {}});

    const CurveMap map(h);
    CHECK(map.build_warnings().total() == 0);
    check_identity_in_space(map);
    // The ends are feature points, still where they were.
    for (const auto& [in, out] : map.feature_points()) {
        REQUIRE(out != kNoVertex);
        CHECK(map.output_vertices()[out] == map.input_vertices()[in]);
    }
}

TEST_CASE("curve-map-arc-length", "[tracking][curves]")
{
    // A bend between two fixed ends: a smoothing step straightens it. The map keeps each point
    // at its fraction of the chain's length; barycentric smoothing keeps its coordinates.
    CurveHistory h;
    h.vertices = {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(3, 0, 0)};
    h.vertex_alive.assign(3, 1);
    h.faces = {{{0, 1}}, {{1, 2}}};
    h.face_alive.assign(2, 1);
    h.edits.push_back({EditKind::move_vertices, {}, {}, {{1, Eigen::Vector3d(1.5, 0, 0)}}});

    const double l0 = std::sqrt(2.), l1 = std::sqrt(5.), total = l0 + l1;
    const CurvePoint p{0, Eigen::Vector2d(0.5, 0.5)}; // half way along the first segment
    {
        const CurveMap map(h);
        const auto q = map.to_output({p})[0];
        const double s = 0.5 * l0 / total; // its fraction of the chain
        CHECK(
            (at(q, map.output_vertices(), map.output_segments()) - Eigen::Vector3d(3 * s, 0, 0))
                .norm() < 1e-12);
        CHECK((map.to_input({q})[0].bary - p.bary).norm() < 1e-12);
    }
    {
        const CurveMap map(h, {tracking::Smoothing::barycentric});
        const auto q = map.to_output({p})[0];
        CHECK(q.segment == p.segment);
        CHECK(q.bary == p.bary);
    }
}

TEST_CASE("curve-map-junction-and-loop", "[tracking][curves]")
{
    // A T: three branches from vertex 0, one of them closing into a loop through vertex 0.
    CurveHistory h;
    h.vertices = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(1, 0, 0),
        Eigen::Vector3d(2, 0, 0),
        Eigen::Vector3d(0, 1, 0),
        Eigen::Vector3d(-1, 1, 0),
        Eigen::Vector3d(-1, 0, 0)};
    h.vertex_alive.assign(6, 1);
    h.faces = {{{0, 1}}, {{1, 2}}, {{0, 3}}, {{3, 4}}, {{4, 5}}, {{5, 0}}};
    h.face_alive.assign(6, 1);
    // The junction moves: each branch maps on its own, by arc length.
    h.edits.push_back({EditKind::move_vertices, {}, {}, {{0, Eigen::Vector3d(0.1, 0.1, 0)}}});
    // Collapse 4 into 3 on the loop: the loop gets shorter, the junction stays.
    h.edits.push_back({EditKind::collapse_edge, {4, 3}, {}, {}});
    // Split the branch to 2.
    h.edits.push_back({EditKind::split_edge, {1, 2, 6}, {1, 6}, {{6, Eigen::Vector3d(1.5, 0, 0)}}});

    const CurveMap map(h);
    CHECK(map.build_warnings().total() == 0);
    for (const bool on_input : {true, false}) {
        const auto pts = random_points(map, on_input, 300);
        const auto there = on_input ? map.to_output(pts) : map.to_input(pts);
        const auto back = on_input ? map.to_input(there) : map.to_output(there);
        for (size_t i = 0; i < pts.size(); ++i) {
            REQUIRE(there[i].segment != kNoVertex);
            REQUIRE(back[i].segment == pts[i].segment);
            CHECK((back[i].bary - pts[i].bary).norm() < 1e-12);
        }
    }
    // The vertex at the far end of the branch to 2 stays where it is: a point there stays there.
    const auto q = map.to_output({{1, Eigen::Vector2d(0, 1)}})[0];
    CHECK(
        (at(q, map.output_vertices(), map.output_segments()) - Eigen::Vector3d(2, 0, 0)).norm() <
        1e-12);
}

TEST_CASE("curve-map-topology-change", "[tracking][curves]")
{
    // Two separate segments welded by a collapse along no segment: the curves' topology
    // changes, which the map reports; points on them still map, by relabelling.
    CurveHistory h;
    h.vertices = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(1, 0, 0),
        Eigen::Vector3d(0, 0.1, 0),
        Eigen::Vector3d(1, 0.1, 0),
        Eigen::Vector3d(2, 0, 0)};
    h.vertex_alive.assign(5, 1);
    h.faces = {{{0, 1}}, {{2, 3}}, {{1, 4}}};
    h.face_alive.assign(3, 1);
    h.edits.push_back({EditKind::collapse_edge, {3, 1}, {}, {}});

    const CurveMap map(h);
    CHECK(map.build_warnings().topology == 1);
    const auto q = map.to_output({{1, Eigen::Vector2d(0.25, 0.75)}})[0];
    CHECK(q.segment == 1);
    CHECK((q.bary - Eigen::Vector2d(0.25, 0.75)).norm() < 1e-12);
}

TEST_CASE("curve-map-weld-and-embed", "[tracking][curves]")
{
    // Files as read: two segments, the second a reversed duplicate of the first after a weld.
    // Then an embed cuts the welded segment in two.
    CurveHistory h;
    h.vertices = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(4, 0, 0),
        Eigen::Vector3d(4, 0, 0),
        Eigen::Vector3d(0, 0, 0)};
    h.vertex_alive.assign(4, 1);
    h.faces = {{{0, 1}}, {{2, 3}}};
    h.face_alive.assign(2, 1);
    h.features = {{{0}}, {{1}}};
    // The weld: 2 -> 1, 3 -> 0; both segments become segment 0.
    h.edits.push_back(
        {EditKind::relabel,
         {0, 1, 1, 0},
         {0, 0},
         {{0, Eigen::Vector3d(0, 0, 0)}, {1, Eigen::Vector3d(4, 0, 0)}}});
    // The embed: (0)-(1) and (1)-(2) over segment 0, with a vertex at x = 1.
    Edit embed{EditKind::embed, {0, 1, 1, 2}, {1, 0, 1, 0, 2, 0, 2}, {}};
    embed.moves = {
        {0, Eigen::Vector3d(0, 0, 0)},
        {1, Eigen::Vector3d(1, 0, 0)},
        {2, Eigen::Vector3d(4, 0, 0)}};
    h.edits.push_back(embed);

    const CurveMap map(h);
    CHECK(map.build_warnings().merged_segments == 1);
    // The second file's segment runs backwards: its point at a quarter is at x = 3.
    const auto q = map.to_output({{1, Eigen::Vector2d(0.75, 0.25)}})[0];
    CHECK(
        (at(q, map.output_vertices(), map.output_segments()) - Eigen::Vector3d(3, 0, 0)).norm() <
        1e-12);
    // Back onto the first file's segment, the one the weld kept.
    const auto p = map.to_input({q})[0];
    CHECK(p.segment == 0);
    CHECK((p.bary - Eigen::Vector2d(0.25, 0.75)).norm() < 1e-12);
    // The feature points: the ends, at the embed's vertices 0 and 2.
    CHECK(map.feature_points() == std::vector<std::array<uint64_t, 2>>{{{0, 0}}, {{1, 2}}});
    CHECK(map.output_feature_points() == std::vector<uint64_t>{0, 2});
}

TEST_CASE("curve-history-file", "[tracking][curves]")
{
    // A curve history saved and loaded replays to the same state; read as a surface, it throws.
    CurveHistory h = polyline({0, 1, 2, 3});
    h.vertices.push_back(Eigen::Vector3d(5, 5, 0)); // a feature point off the curve
    h.vertex_alive.push_back(1);
    h.features = {{{0}}, {{4}}};
    h.edits.push_back({EditKind::split_edge, {0, 1, 5}, {0, 3}, {{5, Eigen::Vector3d(0.5, 0, 0)}}});
    h.edits.push_back({EditKind::collapse_edge, {2, 3}, {}, {}});
    h.edits.push_back({EditKind::consolidate, {}, {}, {}});
    h.output_vertex_ids = {7, 8, 9, 10, 11};
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("wmtk_curve_history_" + std::to_string(std::random_device{}()) + ".wmtkhist");
    h.save(path.string());
    CHECK(history::face_size(path.string()) == 2);
    const CurveHistory loaded = CurveHistory::load(path.string());
    CHECK_THROWS(history::SurfaceHistory::load(path.string()));
    std::filesystem::remove(path);

    history::CurveReplay a(h), b(loaded);
    a.apply_all(h);
    b.apply_all(loaded);
    CHECK(a.faces() == b.faces());
    CHECK(a.face_alive() == b.face_alive());
    CHECK(a.vertices() == b.vertices());
    CHECK(a.features() == b.features());
    CHECK(loaded.output_vertex_ids == h.output_vertex_ids);
    // The feature point off the curve keeps its vertex through the consolidation: 0, 1, 3, 4
    // and the split's 5 stay, so it is the fourth.
    REQUIRE(a.vertices().size() == 5);
    CHECK(a.vertices()[3] == Eigen::Vector3d(5, 5, 0));
    CHECK(a.vertex_alive()[3]);
    CHECK(a.features().count({{3}}) == 1);
}


TEST_CASE("curve-map-junction-moves-along-a-branch", "[tracking][curves]")
{
    // A junction at 2 collapses into its neighbour 1 on one branch: the junction moves, the
    // topology does not change. The branch it moved along gets shorter, the others longer.
    CurveHistory h;
    h.vertices = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(1, 0, 0),
        Eigen::Vector3d(2, 0, 0),
        Eigen::Vector3d(3, 1, 0),
        Eigen::Vector3d(3, -1, 0)};
    h.vertex_alive.assign(5, 1);
    h.faces = {{{0, 1}}, {{1, 2}}, {{2, 3}}, {{2, 4}}};
    h.face_alive.assign(4, 1);
    h.edits.push_back({EditKind::collapse_edge, {2, 1}, {}, {}});
    const CurveMap map(h);
    CHECK(map.build_warnings().total() == 0);
    for (const bool on_input : {true, false}) {
        const auto pts = random_points(map, on_input, 300);
        const auto there = on_input ? map.to_output(pts) : map.to_input(pts);
        const auto back = on_input ? map.to_input(there) : map.to_output(there);
        for (size_t i = 0; i < pts.size(); ++i) {
            REQUIRE(there[i].segment != kNoVertex);
            REQUIRE(back[i].segment == pts[i].segment);
            CHECK((back[i].bary - pts[i].bary).norm() < 1e-12);
        }
    }
    // The junction maps onto the junction.
    const auto q = map.to_output({{2, Eigen::Vector2d(1, 0)}})[0]; // segment (2,3) at 2
    CHECK(
        (at(q, map.output_vertices(), map.output_segments()) - Eigen::Vector3d(1, 0, 0)).norm() <
        1e-12);
}

TEST_CASE("curve-map-end-erodes", "[tracking][curves]")
{
    // An open curve's end collapses into its neighbour: the curve gets shorter, still one open
    // curve. The points between the end and the nearest vertex that stays (1) are squeezed
    // onto what is left of that stretch, the end onto the new end.
    CurveHistory h = polyline({0, 1, 2, 3});
    h.edits.push_back({EditKind::collapse_edge, {3, 2}, {}, {}});
    const CurveMap map(h);
    CHECK(map.build_warnings().total() == 0);
    const auto q = map.to_output({{2, Eigen::Vector2d(0, 1)}})[0]; // the old end, at x = 3
    CHECK(
        (at(q, map.output_vertices(), map.output_segments()) - Eigen::Vector3d(2, 0, 0)).norm() <
        1e-12);
    const auto p = map.to_output({{1, Eigen::Vector2d(0.5, 0.5)}})[0]; // a quarter of 1..3
    CHECK(
        (at(p, map.output_vertices(), map.output_segments()) - Eigen::Vector3d(1.25, 0, 0)).norm() <
        1e-12); // a quarter of 1..2
}

TEST_CASE("curve-map-segment-of-no-length", "[tracking][curves]")
{
    // Vertices 1 and 2 coincide in doubles -- an arrangement can set two apart by less than
    // their rounding -- so segment 1 has no length, until vertex 2 moves away. Its points keep a
    // sliver of the chain, so the map stays one to one: they come back where they were.
    CurveHistory h = polyline({0, 1, 1, 2});
    h.edits.push_back({EditKind::move_vertices, {}, {}, {{2, Eigen::Vector3d(1.5, 0, 0)}}});
    const CurveMap map(h);
    CHECK(map.build_warnings().total() == 0);
    for (const bool on_input : {true, false}) {
        const auto pts = random_points(map, on_input, 300);
        const auto there = on_input ? map.to_output(pts) : map.to_input(pts);
        const auto back = on_input ? map.to_input(there) : map.to_output(there);
        for (size_t i = 0; i < pts.size(); ++i) {
            REQUIRE(there[i].segment != kNoVertex);
            REQUIRE(back[i].segment == pts[i].segment);
            CHECK((back[i].bary - pts[i].bary).norm() < 1e-6);
        }
    }
    // Points on the segment of no length go to distinct places, near vertex 1.
    const auto a = map.to_output({{1, Eigen::Vector2d(0.75, 0.25)}})[0];
    const auto b = map.to_output({{1, Eigen::Vector2d(0.25, 0.75)}})[0];
    const Eigen::Vector3d pa = at(a, map.output_vertices(), map.output_segments());
    const Eigen::Vector3d pb = at(b, map.output_vertices(), map.output_segments());
    CHECK(pa != pb);
    CHECK((pa - Eigen::Vector3d(1, 0, 0)).norm() < 1e-6);
    CHECK((pb - Eigen::Vector3d(1, 0, 0)).norm() < 1e-6);
}

TEST_CASE("curve-map-embed-piece-of-no-length", "[tracking][curves]")
{
    // An embed cuts segment (0, 2) into three pieces, the middle one of no length at x = 1:
    // each piece keeps its own part of the segment, the middle one a sliver.
    CurveHistory h = polyline({0, 2});
    Edit embed{EditKind::embed, {0, 1, 1, 2, 2, 3}, {1, 0, 1, 0, 1, 0, 0}, {}};
    embed.moves = {
        {0, Eigen::Vector3d(0, 0, 0)},
        {1, Eigen::Vector3d(1, 0, 0)},
        {2, Eigen::Vector3d(1, 0, 0)},
        {3, Eigen::Vector3d(2, 0, 0)}};
    h.edits.push_back(embed);
    const CurveMap map(h);
    for (const uint64_t g : {0, 1, 2}) {
        for (const double t : {0.1, 0.5, 0.9}) {
            const CurvePoint p{g, Eigen::Vector2d(1 - t, t)};
            const auto q = map.to_output(map.to_input({p}))[0];
            REQUIRE(q.segment == g);
            CHECK((q.bary - p.bary).norm() < 1e-6);
        }
    }
    // On the pieces of positive length a point keeps its place in space, but for the sliver
    // (a billionth of the segment) the middle piece takes from them.
    const auto q = map.to_input({{2, Eigen::Vector2d(0.5, 0.5)}})[0];
    CHECK(
        (at(q, map.input_vertices(), map.input_segments()) - Eigen::Vector3d(1.5, 0, 0)).norm() <
        1e-8);
}

TEST_CASE("curve-map-maps-from-several-threads", "[tracking][curves]")
{
    // Mapping does not write to the map: threads mapping through one at once each get their own
    // points and their own warnings, those a call alone gets. Every thread's batch has as many
    // points on segments the input does not have as its number, each one unlocated.
    CurveHistory h = polyline({0, 1, 2, 3, 4, 5, 6});
    h.features = {{{0}}, {{6}}};
    h.edits.push_back(
        {EditKind::split_edge, {1, 2, 7}, {1, 6}, {{7, Eigen::Vector3d(1.25, 0, 0)}}});
    h.edits.push_back({EditKind::collapse_edge, {3, 4}, {}, {{4, Eigen::Vector3d(3.5, 0, 0)}}});
    h.edits.push_back({EditKind::move_vertices, {}, {}, {{5, Eigen::Vector3d(4.8, 0.2, 0)}}});
    h.edits.push_back({EditKind::consolidate, {}, {}, {}});
    const CurveMap map(h);
    REQUIRE(map.build_warnings().total() == 0);
    const auto pts = random_points(map, true, 2000);
    std::vector<std::vector<CurvePoint>> batches(8, pts);
    for (size_t t = 0; t < batches.size(); ++t) {
        for (size_t k = 0; k < t; ++k) {
            batches[t].push_back({map.input_segments().size() + k, Eigen::Vector2d(0.5, 0.5)});
        }
    }
    const auto serial = map.to_output(pts);
    std::vector<std::vector<CurvePoint>> there(batches.size());
    std::vector<tracking::CurveMapWarnings> warnings(batches.size());
    {
        std::vector<std::thread> threads;
        for (size_t t = 0; t < batches.size(); ++t) {
            threads.emplace_back([&, t] {
                for (int repeat = 0; repeat < 3; ++repeat) {
                    there[t] = map.to_output(batches[t], &warnings[t]);
                }
            });
        }
        for (auto& t : threads) t.join();
    }
    for (size_t t = 0; t < batches.size(); ++t) {
        INFO("thread " << t);
        CHECK(warnings[t].unlocated == t);
        CHECK(warnings[t].total() == t);
        REQUIRE(there[t].size() == pts.size() + t);
        for (size_t i = 0; i < pts.size(); ++i) {
            CHECK(there[t][i].segment == serial[i].segment);
            CHECK(there[t][i].bary == serial[i].bary);
        }
        for (size_t k = 0; k < t; ++k) CHECK(there[t][pts.size() + k].segment == kNoVertex);
    }
}
