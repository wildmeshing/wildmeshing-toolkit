// triwild's curve history under its options, on the memory route: what recording changes, and
// that the history it returns ends at the mesh it returns.

#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/components/triwild/triwild.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <fstream>
#include <map>
#include <random>
#include <set>
#include <vector>

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

using namespace wmtk;
using namespace wmtk::components;

namespace {

/// A polyline through `v` as an edge mesh, closed or open.
MeshData polyline(const std::vector<Eigen::Vector3d>& v, const bool closed)
{
    MeshData m;
    m.V.resize(Eigen::Index(v.size()), 3);
    for (size_t i = 0; i < v.size(); ++i) m.V.row(Eigen::Index(i)) = v[i].transpose();
    const size_t ne = closed ? v.size() : v.size() - 1;
    auto& E = m.cells[CellType::Edge].connectivity;
    E.resize(Eigen::Index(ne), 2);
    for (size_t i = 0; i < ne; ++i)
        E.row(Eigen::Index(i)) << int64_t(i), int64_t((i + 1) % v.size());
    return m;
}

std::vector<Eigen::Vector3d> circle(const int n, const double r = 1, const double cx = 0)
{
    std::vector<Eigen::Vector3d> v;
    for (int i = 0; i < n; ++i) {
        const double a = 2 * M_PI * i / n;
        v.push_back(Eigen::Vector3d(cx + r * std::cos(a), r * std::sin(a), 0));
    }
    return v;
}

std::vector<Eigen::Vector3d> wave(const int m)
{
    std::vector<Eigen::Vector3d> v;
    for (int i = 0; i <= m; ++i) {
        const double x = -1.6 + 3.2 * i / m;
        v.push_back(Eigen::Vector3d(x, 0.3 * std::sin(6 * x) + 0.1, 0));
    }
    return v;
}

/// The worked example's curves: a circle and an open wave crossing it twice.
MeshMap example()
{
    return {{"circle", polyline(circle(300), true)}, {"wave", polyline(wave(100), false)}};
}

/// triwild on the example, recording unless `record` is false, with `extra` options.
ComponentResult run(const nlohmann::json& extra, const bool record = true)
{
    nlohmann::json options = {
        {"input", {"circle", "wave"}},
        {"length_rel", 0.02},
        {"max_iterations", 10},
        {"num_threads", 1}};
    if (record) options["history_output"] = "record";
    if (!extra.is_null()) options.update(extra);
    return triwild::triwild(options, example());
}

history::CurveHistory history_of(const ComponentResult& r)
{
    return history::CurveHistory::from_bytes(r.second.at("history").get_binary());
}

/// The parameters the history recorded.
nlohmann::json parameters_of(const ComponentResult& r)
{
    return nlohmann::json::parse(history_of(r).metadata);
}

/// A number in [0, 1) from the generator alone: mt19937 gives the same numbers everywhere, but
/// std::uniform_real_distribution does not -- libstdc++ and libc++ draw them differently -- and
/// the points mapped here must not depend on the standard library.
double uniform(std::mt19937& rng)
{
    return double(rng()) / 4294967296.0;
}

using Ends = std::array<std::array<double, 2>, 2>;
/// An edge by its ends' positions, the smaller first.
Ends ends_of(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    Ends e{{{{a.x(), a.y()}}, {{b.x(), b.y()}}}};
    if (e[1] < e[0]) std::swap(e[0], e[1]);
    return e;
}

/// What the map of a history does with random points on its input.
struct Mapped
{
    size_t points = 0, lost = 0;
    /// The largest error of a mapped point back on the input, and how many came back within
    /// 1e-8 of where they started.
    double round_trip = 0;
    size_t exact = 0;
};

/**
 * @brief What every history triwild returns must hold, whatever its options: it starts at the
 * `n_segments` segments of the curves given, and ends at the mesh returned -- its last state's
 * segments, by output_vertex_ids, are edges of the mesh at the same positions, and exactly the
 * edges of the tracked curves ("surface"). Returns what the map does with points on the input.
 */
Mapped check_history(const ComponentResult& r, const size_t n_segments)
{
    const history::CurveHistory h = history_of(r);
    const tracking::CurveTracking t(h);
    CHECK(t.input_segments().size() == n_segments);
    REQUIRE(t.output_by_vertices());
    const MeshData& mesh = r.first.at("mesh");
    const auto V = [&mesh](const uint64_t v) {
        return Eigen::Vector3d(mesh.V(Eigen::Index(v), 0), mesh.V(Eigen::Index(v), 1), 0);
    };
    std::set<std::array<uint64_t, 2>> mesh_edges;
    const auto& F = mesh.cell(CellType::Triangle).connectivity;
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            const uint64_t x = uint64_t(F(f, k)), y = uint64_t(F(f, (k + 1) % 3));
            mesh_edges.insert({{std::min(x, y), std::max(x, y)}});
        }
    }
    // The last state's vertices are the mesh's, where they are.
    const auto& out = t.map().output_vertices();
    for (uint64_t v = 0; v < h.output_vertex_ids.size(); ++v) {
        const uint64_t o = h.output_vertex_ids[v];
        if (o == history::kNoVertex) continue;
        REQUIRE(o < uint64_t(mesh.V.rows()));
        CHECK(Eigen::Vector3d(out[v].x(), out[v].y(), 0) == V(o));
    }
    std::set<Ends> tracked, surface;
    for (const auto& e : t.output_segments()) {
        CHECK(mesh_edges.count({{std::min(e[0], e[1]), std::max(e[0], e[1])}}) == 1);
        tracked.insert(ends_of(V(e[0]), V(e[1])));
    }
    const MeshData& s = r.first.at("surface");
    const auto& E = s.cell(CellType::Edge).connectivity;
    for (Eigen::Index i = 0; i < E.rows(); ++i) {
        surface.insert(ends_of(
            Eigen::Vector3d(s.V(E(i, 0), 0), s.V(E(i, 0), 1), 0),
            Eigen::Vector3d(s.V(E(i, 1), 0), s.V(E(i, 1), 1), 0)));
    }
    CHECK(tracked == surface);

    // Random points on the input, to the output and back.
    std::mt19937 rng(5);
    std::vector<tracking::InputCurvePoint> pts;
    for (int i = 0; i < 1000; ++i) {
        const double x = uniform(rng);
        pts.push_back({uint64_t(rng() % n_segments), Eigen::Vector2d(1 - x, x)});
    }
    const auto there = t.to_output(pts);
    const auto back = t.to_input(there);
    Mapped m;
    m.points = pts.size();
    for (size_t i = 0; i < pts.size(); ++i) {
        if (there[i].vertices[0] == tracking::kNoId) {
            ++m.lost;
            continue;
        }
        REQUIRE(back[i].segment != tracking::kNoId);
        double err = 0;
        if (back[i].segment == pts[i].segment) {
            err = (back[i].bary - pts[i].bary).norm();
        } else {
            // A duplicate segment maps back onto the one it duplicates: the same place.
            const auto& S = t.input_segments();
            const auto& IV = t.map().input_vertices();
            const auto at = [&](const tracking::InputCurvePoint& p) {
                return Eigen::Vector3d(
                    p.bary[0] * IV[S[p.segment][0]] + p.bary[1] * IV[S[p.segment][1]]);
            };
            err = (at(back[i]) - at(pts[i])).norm();
        }
        m.round_trip = std::max(m.round_trip, err);
        m.exact += err < 1e-8;
    }
    return m;
}

} // namespace

TEST_CASE("triwild-history-anchors-junctions", "[triwild][tracking]" + release_only)
{
    // Recording anchors junctions -- allow_junction_cleanup defaults to false -- unless the
    // config gives allow_junction_cleanup, or preserve_feature_points is off: that anchors
    // nothing, junctions or ends, so recording leaves the option and the output alone.
    CHECK_FALSE(parameters_of(run({})).at("allow_junction_cleanup").get<bool>());
    CHECK(parameters_of(run({{"allow_junction_cleanup", true}})).at("allow_junction_cleanup"));
    const nlohmann::json no_features = {{"preserve_feature_points", false}};
    const ComponentResult recorded = run(no_features);
    CHECK(parameters_of(recorded).at("allow_junction_cleanup"));
    const ComponentResult plain = run(no_features, false);
    CHECK_FALSE(plain.second.contains("history"));
    const MeshData& a = recorded.first.at("mesh");
    const MeshData& b = plain.first.at("mesh");
    CHECK(a.V == b.V);
    CHECK(a.cell(CellType::Triangle).connectivity == b.cell(CellType::Triangle).connectivity);
}

namespace {

/// triwild at default topology on the example and a third curve, `offset` above the wave where
/// x < `x_max`: closer than eps, so that the simplification may weld it to the wave. Also
/// returns the number of segments of the three.
std::pair<ComponentResult, size_t> run_near(const double offset, const double x_max)
{
    MeshMap meshes = example();
    std::vector<Eigen::Vector3d> near;
    for (const Eigen::Vector3d& p : wave(100)) {
        if (p.x() < x_max) near.push_back(p + Eigen::Vector3d(0, offset, 0));
    }
    meshes.emplace("near", polyline(near, false));
    const ComponentResult r = triwild::triwild(
        {{"input", {"circle", "wave", "near"}},
         {"history_output", "record"},
         {"length_rel", 0.02},
         {"max_iterations", 10},
         {"num_threads", 1}},
        meshes);
    return {r, 300 + 100 + near.size() - 1};
}

} // namespace

TEST_CASE("triwild-history-default-topology", "[triwild][tracking]" + release_only)
{
    // Without preserve_topology the simplification may weld curves that pass within the
    // envelope: a third curve runs along the wave, closer than eps. The weld and the segments it
    // merges are warnings of the map, which still ends at the mesh and maps every point.
    const auto [r, n] = run_near(2e-4, -0.8);
    const Mapped m = check_history(r, n);
    // Two curves welded into one: their points map onto it, and back onto one of the two.
    CHECK(m.lost == 0);
    INFO(m.exact << " of " << m.points << " exact, worst " << m.round_trip);
    CHECK(m.exact > m.points * 9 / 10);
    CHECK(m.exact < m.points);
    const tracking::CurveTracking t(history_of(r));
    CHECK(t.map().build_warnings().topology > 0);
}

TEST_CASE("triwild-history-contracted-chains", "[triwild][tracking]" + release_only)
{
    // The third curve closer and longer: its crossings with the circle lie next to the wave's,
    // and the collapses contract the curves between them. Two junctions collapse into one, the
    // segment between them shrinking to a point; loops of three segments close, two of them
    // merging into the third. Neither leaves a chain to pair with: each maps onto what it
    // became, the point or the third segment, and every point still has an image -- the
    // collapsed segments' points had none before, about one in a hundred here.
    const auto [r, n] = run_near(1e-4, -1);
    const Mapped m = check_history(r, n);
    CHECK(m.lost == 0);
    CHECK(m.exact < m.points);
    const tracking::CurveTracking t(history_of(r));
    CHECK(t.map().build_warnings().topology > 0);
    CHECK(t.map().build_warnings().merged_segments > 0);
}

TEST_CASE("triwild-history-preserve-topology", "[triwild][tracking]" + release_only)
{
    // The same curves under preserve_topology: no topology warning, and the map a bijection.
    const ComponentResult r = run({{"preserve_topology", true}});
    const Mapped m = check_history(r, 400);
    CHECK(m.lost == 0);
    CHECK(m.round_trip < 1e-8);
    CHECK(tracking::CurveTracking(history_of(r)).map().build_warnings().total() == 0);
}

TEST_CASE("triwild-history-filters", "[triwild][tracking]" + release_only)
{
    // A filter removes triangles after the optimization, and the tracked edges with them that no
    // triangle holds any more: the history records that as segments replaced by nothing
    // (resync, replace_faces), and ends at the filtered mesh. Points that were on them have no
    // image; every other point maps there and back.
    for (const std::string filter : {"input", "flood"}) {
        INFO("filter " << filter);
        const ComponentResult r = run({{"filter", filter}, {"preserve_topology", true}});
        const Mapped m = check_history(r, 400);
        CHECK(m.lost > 0);
        CHECK(m.lost < m.points);
        CHECK(m.round_trip < 1e-8);
        const tracking::CurveTracking t(history_of(r));
        CHECK(t.map().build_warnings().replaced > 0);
        CHECK(t.map().build_warnings().topology == 0);
    }
}

TEST_CASE("triwild-history-free-junctions", "[triwild][tracking]" + release_only)
{
    // With allow_junction_cleanup the junctions may slide: the map follows them, a bijection
    // still under preserve_topology.
    const ComponentResult r = run({{"allow_junction_cleanup", true}, {"preserve_topology", true}});
    CHECK(parameters_of(r).at("allow_junction_cleanup"));
    const Mapped m = check_history(r, 400);
    CHECK(m.lost == 0);
    CHECK(m.round_trip < 1e-8);
}

TEST_CASE("triwild-history-skip-simplify", "[triwild][tracking]" + release_only)
{
    // Without the input simplification the history goes from the curves as read to the
    // arrangement directly: a weld, then the embed.
    const ComponentResult r = run({{"skip_simplify", true}, {"preserve_topology", true}});
    const history::CurveHistory h = history_of(r);
    REQUIRE(h.edits.size() >= 2);
    CHECK(h.edits[0].kind == history::EditKind::relabel);
    CHECK(h.edits[1].kind == history::EditKind::embed);
    const Mapped m = check_history(r, 400);
    CHECK(m.lost == 0);
    CHECK(m.round_trip < 1e-8);
}

TEST_CASE("triwild-history-duplicates", "[triwild][tracking]")
{
    // A polyline that comes back over itself, 1e-7 away, which remove_duplicate_eps welds: the
    // history starts at the segments as given, every one of them, and the weld makes each
    // duplicate the segment it duplicates -- a point on either goes to the same place.
    std::vector<Eigen::Vector3d> v;
    for (int i = 0; i <= 20; ++i) {
        v.push_back(Eigen::Vector3d(-1 + 0.1 * i, 0.2 * std::sin(3.0 * i / 20), 0));
    }
    for (int i = 19; i >= 10; --i) v.push_back(v[size_t(i)] + Eigen::Vector3d(1e-7, 0, 0));
    const MeshMap meshes{{"curve", polyline(v, false)}};
    const ComponentResult r = triwild::triwild(
        {{"input", {"curve"}},
         {"history_output", "record"},
         {"remove_duplicate_eps", 1e-5},
         {"length_rel", 0.05},
         {"max_iterations", 5},
         {"num_threads", 1}},
        meshes);
    const size_t n = v.size() - 1;
    const Mapped m = check_history(r, n);
    CHECK(m.lost == 0);
    CHECK(m.round_trip < 1e-6);
    // The weld: the segments back, 20 to 29, are the ones out, 19 down to 10.
    const history::CurveHistory h = history_of(r);
    REQUIRE(h.edits[0].kind == history::EditKind::relabel);
    const auto& faces = h.edits[0].faces;
    REQUIRE(faces.size() == n);
    for (size_t k = 0; k < 10; ++k) CHECK(faces[20 + k] == faces[19 - k]);
    const tracking::CurveTracking t(h);
    CHECK(t.map().build_warnings().merged_segments > 0);
    const auto a = t.to_output({{19, Eigen::Vector2d(0.3, 0.7)}})[0];
    const auto b = t.to_output({{20, Eigen::Vector2d(0.7, 0.3)}})[0];
    REQUIRE(a.vertices[0] != tracking::kNoId);
    const auto& V = r.first.at("mesh").V;
    const auto at = [&V](const tracking::OutputCurvePoint& p) {
        return Eigen::Vector3d(
            p.bary[0] * V.row(Eigen::Index(p.vertices[0])).transpose() +
            p.bary[1] * V.row(Eigen::Index(p.vertices[1])).transpose());
    };
    CHECK((at(a) - at(b)).norm() < 1e-9);
}

TEST_CASE("triwild-history-file-and-memory-routes", "[triwild][tracking]" + release_only)
{
    // The same curves through files and in memory record the same history: the same first
    // state, the same edits, the same output ids. Only the parameters differ, the route among
    // them.
    const std::filesystem::path dir =
        std::filesystem::current_path() / "triwild_history_file_and_memory";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    // The example's curves as OBJ polylines, the files read_edge_mesh reads.
    for (const auto& [name, m] : example()) {
        std::ofstream o(dir / (name + ".obj"));
        o.precision(17);
        for (Eigen::Index v = 0; v < m.V.rows(); ++v)
            o << "v " << m.V(v, 0) << " " << m.V(v, 1) << " 0\n";
        const auto& E = m.cell(CellType::Edge).connectivity;
        for (Eigen::Index e = 0; e < E.rows(); ++e)
            o << "l " << E(e, 0) + 1 << " " << E(e, 1) + 1 << "\n";
    }
    const nlohmann::json options = {
        {"preserve_topology", true},
        {"length_rel", 0.02},
        {"max_iterations", 10},
        {"num_threads", 1}};
    nlohmann::json file = options;
    file.update({
        {"application", "triwild"},
        {"input", {(dir / "circle.obj").string(), (dir / "wave.obj").string()}},
        {"output", (dir / "out").string()},
        {"history_output", (dir / "out.wmtkhist").string()},
        {"write_vtu", false},
    });
    triwild::triwild(file);
    const auto a = history::CurveHistory::load((dir / "out.wmtkhist").string());
    const ComponentResult r = run(options);
    const history::CurveHistory b = history_of(r);
    CHECK(a.vertices == b.vertices);
    CHECK(a.faces == b.faces);
    CHECK(a.features == b.features);
    CHECK(a.output_vertex_ids == b.output_vertex_ids);
    REQUIRE(a.edits.size() == b.edits.size());
    for (size_t i = 0; i < a.edits.size(); ++i) {
        CHECK(a.edits[i].kind == b.edits[i].kind);
        CHECK(a.edits[i].v == b.edits[i].v);
        CHECK(a.edits[i].faces == b.edits[i].faces);
    }
    CHECK(nlohmann::json::parse(a.metadata).at("route") == "file");
    CHECK(nlohmann::json::parse(b.metadata).at("route") == "memory");
    // And the output the file route wrote is the mesh the memory route returned.
    CHECK(io::read_mesh(dir / "out.msh").V == r.first.at("mesh").V);
    std::filesystem::remove_all(dir);
}
