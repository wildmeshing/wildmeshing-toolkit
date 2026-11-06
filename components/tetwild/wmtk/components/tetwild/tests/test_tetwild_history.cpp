#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/components/tracking/SurfaceMap.hpp>
#include <wmtk/components/tracking/surface_io.hpp>
#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/components/tracking/transfer.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <igl/readOBJ.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <random>
#include <set>

namespace fs = std::filesystem;
using namespace wmtk;
using namespace wmtk::components;

namespace {

/// Whole runs of the application, too slow for the Debug CI jobs: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

/// A face by its corner positions, sorted: the same in the history and in the exported mesh.
using Key = std::array<std::array<double, 3>, 3>;

Key key_of(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
    Key k = {{{{a[0], a[1], a[2]}}, {{b[0], b[1], b[2]}}, {{c[0], c[1], c[2]}}}};
    std::sort(k.begin(), k.end());
    return k;
}

} // namespace

TEST_CASE(
    "tetwild-surface-history-replays-to-the-tracked-surface",
    "[tetwild][history]" + release_only)
{
    // Two spheres, from two files: the history runs from the files as read -- the second a
    // triangle soup, every vertex repeated per face, so the weld has work to do -- through the
    // insertion and the optimization to the output's tracked surface. It ends exactly there,
    // the map is a bijection, and an output face maps onto the file of the sphere it is on.
    // Under preserve_topology: without it, a run may weld or merge a few faces, and the map is
    // then rightly not a bijection there. A serial run is the same on every platform and is
    // held to that exactly; a parallel one, whose operations the schedule orders, may lose or
    // misplace one face in a hundred.
    const fs::path input = fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj";
    REQUIRE(fs::exists(input));
    const fs::path dir = fs::current_path() / "tetwild_history_test";
    fs::create_directories(dir);
    Eigen::MatrixXd SV;
    Eigen::MatrixXi SF;
    REQUIRE(igl::readOBJ(input.string(), SV, SF));
    const Eigen::RowVector3d shift(3, 0, 0);
    const fs::path soup = dir / "sphere_soup.obj";
    {
        std::ofstream o(soup);
        for (Eigen::Index f = 0; f < SF.rows(); ++f) {
            for (int k = 0; k < 3; ++k) {
                const Eigen::RowVector3d p = SV.row(SF(f, k)) + shift;
                o << fmt::format("v {:.17g} {:.17g} {:.17g}\n", p[0], p[1], p[2]);
            }
        }
        for (Eigen::Index f = 0; f < SF.rows(); ++f) {
            o << "f " << 3 * f + 1 << ' ' << 3 * f + 2 << ' ' << 3 * f + 3 << '\n';
        }
    }
    const Eigen::Vector3d centre0 = 0.5 * (SV.colwise().minCoeff() + SV.colwise().maxCoeff());
    const Eigen::Vector3d centre1 = centre0 + shift.transpose();
    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        const fs::path out = dir / fmt::format("spheres_{}", threads);
        const fs::path hist = dir / fmt::format("spheres_{}.wmtkhist", threads);
        const auto exported = tetwild::tetwild_with_export({
            {"application", "tetwild"},
            {"input", {input.string(), soup.string()}},
            {"output", out.string()},
            {"history_output", hist.string()},
            {"max_iterations", 5},
            {"num_threads", threads},
            {"preserve_topology", true},
            {"write_vtu", false},
        });
        const auto h = history::SurfaceHistory::load(hist.string());
        const tracking::SurfaceMap map(h);
        // It starts at the files: every face of both, the soup's vertices not yet welded.
        CHECK(map.input_faces().size() == size_t(2 * SF.rows()));
        CHECK(map.input_vertices().size() == size_t(SV.rows() + 3 * SF.rows()));

        std::set<Key> tracked;
        for (Eigen::Index f = 0; f < exported.F.rows(); ++f) {
            tracked.insert(key_of(
                exported.V.row(exported.F(f, 0)).transpose(),
                exported.V.row(exported.F(f, 1)).transpose(),
                exported.V.row(exported.F(f, 2)).transpose()));
        }
        std::set<Key> replayed;
        const auto& V = map.output_vertices();
        for (size_t f = 0; f < map.output_faces().size(); ++f) {
            if (!map.output_face_alive()[f]) continue;
            const auto& t = map.output_faces()[f];
            replayed.insert(key_of(V[t[0]], V[t[1]], V[t[2]]));
        }
        CHECK(replayed == tracked);

        std::vector<tracking::SurfacePoint> pts;
        for (uint64_t f = 0; f < map.output_faces().size(); ++f) {
            if (map.output_face_alive()[f]) pts.push_back({f, Eigen::Vector3d::Constant(1. / 3)});
        }
        tracking::MapWarnings back_warnings;
        const auto there = map.to_input(pts);
        const auto back = map.to_output(there, &back_warnings);
        size_t lost = 0, wrong_file = 0;
        double err = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            const auto& t = map.output_faces()[pts[i].face];
            const Eigen::Vector3d c = (V[t[0]] + V[t[1]] + V[t[2]]) / 3;
            const bool second = (c - centre1).norm() < (c - centre0).norm();
            if (there[i].face >= map.input_faces().size() || back[i].face != pts[i].face) {
                ++lost;
                continue;
            }
            wrong_file += (there[i].face >= uint64_t(SF.rows())) != second;
            err = std::max(err, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
        }
        const auto& w = map.build_warnings();
        INFO(
            "build warnings " << w.total() << " (not a disk " << w.not_a_disk << ", flat "
                              << w.flat_triangle << ", inverted " << w.inverted << ", merged "
                              << w.merged_faces << ", replaced " << w.replaced << "), " << lost
                              << " lost, " << wrong_file << " on the wrong file, of " << pts.size()
                              << "; round trip " << err);
        if (threads == 0) {
            CHECK(lost == 0);
            CHECK(wrong_file == 0);
            CHECK(w.total() == 0);
            CHECK(back_warnings.total() == 0);
            CHECK(err < 1e-8);
        } else {
            CHECK(100 * (lost + wrong_file) <= pts.size());
        }
    }
}

TEST_CASE("tetwild-surface-history-octocat", "[tetwild][history][.]")
{
    // A real input, welded, simplified, inserted and optimized with preserve_topology, which
    // holds the simplification to the link condition too: the topology the map needs -- unless
    // the simplification folds the surface through itself, which the insertion then cuts. At 8
    // threads that happens in about 1 run in 3 to 5 here (2026-11), the Euler characteristic
    // [2] -> [3]. So by phase: the optimization keeps the inserted surface's topology always; the
    // input's topology and the map are checked only when the inserted surface does not cross
    // itself.
    const fs::path input = fs::path(WMTK_DATA_DIR) / "models" / "Octocat.obj";
    REQUIRE(fs::exists(input));
    const fs::path dir = fs::current_path() / "tetwild_history_test";
    fs::create_directories(dir);
    const fs::path hist = dir / "octocat.wmtkhist";
    const fs::path report_file = dir / "octocat_report.json";
    tetwild::tetwild_with_export({
        {"application", "tetwild"},
        {"input", {input.string()}},
        {"output", (dir / "octocat").string()},
        {"history_output", hist.string()},
        {"report", report_file.string()},
        {"num_threads", 8},
        {"preserve_topology", true},
        {"write_vtu", false},
    });
    nlohmann::json report;
    std::ifstream report_in(report_file);
    report_in >> report;
    CHECK(report["inserted_euler_characteristic"] == report["output_euler_characteristic"]);
    const int64_t crossing_pairs = report["inserted_crossing_pairs"];
    const int64_t degenerate = report["inserted_degenerate_triangles"];
    if (crossing_pairs != 0 || degenerate != 0) {
        WARN(
            "the inserted surface crosses itself, " << crossing_pairs << " crossing pairs and "
                                                    << degenerate << " degenerate triangles: "
                                                    << "the input's topology and the map are "
                                                    << "not checked");
        return;
    }
    CHECK(report["input_euler_characteristic"] == report["output_euler_characteristic"]);
    const tracking::SurfaceMap map(history::SurfaceHistory::load(hist.string()));
    std::vector<tracking::SurfacePoint> pts;
    for (uint64_t f = 0; f < map.output_faces().size(); ++f) {
        if (map.output_face_alive()[f]) pts.push_back({f, Eigen::Vector3d::Constant(1. / 3)});
    }
    tracking::MapWarnings back_warnings;
    const auto back = map.to_output(map.to_input(pts), &back_warnings);
    double err = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(back[i].face == pts[i].face);
        err = std::max(err, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
    }
    CHECK(map.build_warnings().total() == 0);
    CHECK(back_warnings.total() == 0);
    CHECK(err < 1e-8);
}

TEST_CASE(
    "tetwild-surface-history-transfers-onto-the-tet-mesh",
    "[tetwild][history]" + release_only)
{
    // Per-face labels on the input, carried to the tracked surface of the output and written into
    // the tet mesh tetwild wrote: onto its triangles, its face tags on the tet mesh's own nodes,
    // each with the label of the input face it maps to.
    const fs::path dir = fs::current_path() / "tetwild_history_test";
    fs::create_directories(dir);
    Eigen::MatrixXd SV;
    Eigen::MatrixXi SF;
    REQUIRE(igl::readOBJ((fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj").string(), SV, SF));
    tracking::SurfaceData labelled;
    for (Eigen::Index v = 0; v < SV.rows(); ++v) labelled.vertices.push_back(SV.row(v).transpose());
    auto& label = labelled.face_fields["label"];
    for (Eigen::Index f = 0; f < SF.rows(); ++f) {
        labelled.faces.push_back({{uint64_t(SF(f, 0)), uint64_t(SF(f, 1)), uint64_t(SF(f, 2))}});
        label.push_back({double(f % 7)});
    }
    const fs::path input = dir / "sphere_labels.msh";
    tracking::write_surface_msh(input.string(), labelled);

    const fs::path out = dir / "labels";
    const fs::path hist = dir / "labels.wmtkhist";
    const auto exported = tetwild::tetwild_with_export({
        {"application", "tetwild"},
        {"input", {input.string()}},
        {"output", out.string()},
        {"history_output", hist.string()},
        {"max_iterations", 3},
        {"write_vtu", false},
    });
    const auto h = history::SurfaceHistory::load(hist.string());
    const tracking::SurfaceMap map(h);
    const auto in = tracking::read_surface({input.string()});
    std::string why;
    REQUIRE(tracking::matches_input(map, in, why));
    const auto transferred = tracking::transfer_fields(map, in);
    REQUIRE(transferred.faces.size() == size_t(exported.F.rows()));
    const fs::path with_labels = dir / "labels_final_with_labels.msh";
    tracking::write_surface_into_msh(
        out.string() + "_final.msh",
        with_labels.string(),
        transferred,
        tracking::output_rows(map, h.output_vertex_ids));

    const MeshData msh = io::read_mesh(with_labels);
    CHECK(msh.cell(CellType::Tetrahedron).connectivity.rows() == exported.T.rows());
    const CellData& triangles = msh.cell(CellType::Triangle);
    // tetwild writes its tracked faces, its face tags, as the triangle block: each face of the
    // surface lands on the row of its own face, and no row is added.
    REQUIRE(size_t(triangles.connectivity.rows()) == transferred.faces.size());
    // Its vertices are the tet mesh's, where the surface has its vertices.
    std::set<Key> rows;
    for (Eigen::Index r = 0; r < triangles.connectivity.rows(); ++r) {
        const auto corner = [&](int k) -> Eigen::Vector3d {
            return msh.V.row(triangles.connectivity(r, k)).transpose();
        };
        rows.insert(key_of(corner(0), corner(1), corner(2)));
    }
    size_t misplaced = 0, mislabelled = 0;
    for (const auto& f : transferred.faces) {
        misplaced += rows.count(key_of(
                         transferred.vertices[f[0]],
                         transferred.vertices[f[1]],
                         transferred.vertices[f[2]])) == 0;
    }
    const bool found = triangles.attributes.count("label") > 0;
    if (found) {
        const auto labels = triangles.attributes.at("label").doubles();
        const auto sources = triangles.attributes.at("input_face").integers();
        for (Eigen::Index r = 0; r < triangles.connectivity.rows(); ++r) {
            REQUIRE(sources(r, 0) >= 0);
            mislabelled += labels(r, 0) != label[size_t(sources(r, 0))][0];
        }
    }
    CHECK(found);
    CHECK(misplaced == 0);
    CHECK(mislabelled == 0);
}

// tetwild's curve history (curve_history_output) under its options, on the memory route: what
// recording changes, and that the history it returns ends at the mesh it returns. The 3D twin of
// triwild's test_triwild_history.cpp.
namespace {

/// The cube [0,2]^3 as a triangle mesh.
MeshData cube()
{
    MeshData m;
    m.V.resize(8, 3);
    m.V << 0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 2, 2, 0, 2, 2, 2, 2, 0, 2, 2;
    auto& F = m.cells[CellType::Triangle].connectivity;
    F.resize(12, 3);
    F << 0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 2, 3, 7, 2, 7, 6, 1, 2, 6, 1, 6, 5,
        0, 4, 7, 0, 7, 3;
    return m;
}

/// An open polyline through `v`, or the points `v` with no segment.
MeshData curve(const std::vector<Eigen::Vector3d>& v, const bool segments = true)
{
    MeshData m;
    m.V.resize(Eigen::Index(v.size()), 3);
    for (size_t i = 0; i < v.size(); ++i) m.V.row(Eigen::Index(i)) = v[i].transpose();
    auto& E = m.cells[CellType::Edge].connectivity;
    E.resize(segments ? Eigen::Index(v.size()) - 1 : 0, 2);
    for (Eigen::Index i = 0; i < E.rows(); ++i) E.row(i) << i, i + 1;
    return m;
}

/// The bent chain inside the cube, of 4 segments, `dy` off it along y.
std::vector<Eigen::Vector3d> chain(const double dy = 0)
{
    std::vector<Eigen::Vector3d> v =
        {{0.5, 0.5, 0.5}, {0.75, 0.75, 0.75}, {1, 1, 1}, {1.25, 1, 1.25}, {1.5, 1, 1.5}};
    for (auto& p : v) p.y() += dy;
    return v;
}

/// The worked example's input: the cube, and three feature curve files -- the chain inside it, a
/// segment on its face y = 0, a chain outside it, past x = 2 -- of 8 segments, and three points:
/// inside, on the face x = 2, at the corner (0, 0, 0).
MeshMap curve_scene()
{
    return {
        {"cube", cube()},
        {"chain", curve(chain())},
        {"rim", curve({{0.5, 0, 0.5}, {1, 0, 1}, {1.5, 0, 1.5}})},
        {"outside", curve({{2.3, 0.5, 1}, {2.3, 1, 1}, {2.3, 1.5, 1}})},
        {"points", curve({{1, 0.5, 1.5}, {2, 1, 1}, {0, 0, 0}}, false)}};
}
constexpr size_t n_curve_segments = 8;
constexpr size_t n_curve_features = 9; // the ends of the three curves, and the points

/// tetwild on the example, recording the curves unless `record` is false, with `extra` options.
/// With `near`, a fourth curve input: the chain, `near` off it.
ComponentResult run_curves(const nlohmann::json& extra, const bool record = true, double near = 0)
{
    nlohmann::json options = {
        {"input", {"cube", "chain", "rim", "outside", "points"}},
        {"max_iterations", 3}};
    MeshMap meshes = curve_scene();
    if (near != 0) {
        meshes.emplace("near", curve(chain(near)));
        options["input"].push_back("near");
    }
    if (record) options["curve_history_output"] = "record";
    if (!extra.is_null()) options.update(extra);
    return tetwild::tetwild(options, meshes);
}

/// `m` as an OBJ file: its vertices, triangles and segments, the files tetwild's file route reads.
void write_obj(const fs::path& path, const MeshData& m)
{
    std::ofstream o(path);
    for (Eigen::Index v = 0; v < m.V.rows(); ++v) {
        o << fmt::format("v {:.17g} {:.17g} {:.17g}\n", m.V(v, 0), m.V(v, 1), m.V(v, 2));
    }
    for (const auto& [type, c] : m.cells) {
        const char* tag = type == CellType::Triangle ? "f" : "l";
        for (Eigen::Index i = 0; i < c.connectivity.rows(); ++i) {
            o << tag;
            for (Eigen::Index k = 0; k < c.connectivity.cols(); ++k) {
                o << ' ' << c.connectivity(i, k) + 1;
            }
            o << '\n';
        }
    }
}

history::CurveHistory curve_history_of(const ComponentResult& r)
{
    return history::CurveHistory::from_bytes(r.second.at("curve_history").get_binary());
}

/// What the map of a curve history does with random points on its input.
struct CurveMapped
{
    size_t points = 0, lost = 0;
    double round_trip = 0;
    size_t exact = 0;
    size_t features_kept = 0;
};

/**
 * @brief What every curve history tetwild returns must hold, whatever its options: it starts at
 * the segments of the curve files, and ends at the mesh returned -- its last state's segments,
 * by output_vertex_ids, are edges of the tet mesh at the same positions, and exactly the edges
 * of its edge block, the feature curves. Returns what the map does with points on the input,
 * and how many feature points end within the anchors' radius of where they were.
 */
CurveMapped check_curve_history(
    const ComponentResult& r,
    const size_t n_segments = n_curve_segments,
    const size_t n_features = n_curve_features)
{
    const history::CurveHistory h = curve_history_of(r);
    const tracking::CurveTracking t(h);
    CHECK(t.input_segments().size() == n_segments);
    REQUIRE(t.output_by_vertices());
    const MeshData& mesh = r.first.at("mesh");
    const auto V = [&mesh](const uint64_t v) {
        return Eigen::Vector3d(mesh.V.row(Eigen::Index(v)).transpose());
    };
    std::set<std::array<uint64_t, 2>> tet_edges;
    const auto& T = mesh.cell(CellType::Tetrahedron).connectivity;
    for (Eigen::Index t = 0; t < T.rows(); ++t) {
        for (int a = 0; a < 4; ++a) {
            for (int b = a + 1; b < 4; ++b) {
                const uint64_t x = uint64_t(T(t, a)), y = uint64_t(T(t, b));
                tet_edges.insert({{std::min(x, y), std::max(x, y)}});
            }
        }
    }
    const auto& out = t.map().output_vertices();
    for (uint64_t v = 0; v < h.output_vertex_ids.size(); ++v) {
        const uint64_t o = h.output_vertex_ids[v];
        if (o == history::kNoVertex) continue;
        REQUIRE(o < uint64_t(mesh.V.rows()));
        CHECK(out[v] == V(o));
    }
    std::set<std::array<uint64_t, 2>> tracked, block;
    for (const auto& e : t.output_segments()) {
        const std::array<uint64_t, 2> s = {{std::min(e[0], e[1]), std::max(e[0], e[1])}};
        CHECK(tet_edges.count(s) == 1);
        tracked.insert(s);
    }
    const auto& E = mesh.cell(CellType::Edge).connectivity;
    for (Eigen::Index i = 0; i < E.rows(); ++i) {
        const uint64_t x = uint64_t(E(i, 0)), y = uint64_t(E(i, 1));
        block.insert({{std::min(x, y), std::max(x, y)}});
    }
    CHECK(tracked == block);

    std::mt19937 rng(5);
    std::vector<tracking::InputCurvePoint> pts;
    for (int i = 0; i < 1000; ++i) {
        const double x = double(rng()) / 4294967296.0;
        pts.push_back({uint64_t(rng() % n_segments), Eigen::Vector2d(1 - x, x)});
    }
    const auto there = t.to_output(pts);
    const auto back = t.to_input(there);
    CurveMapped m;
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
            // A segment welded onto another maps back onto that one: the error is in space.
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
    // The anchors' radius: feature_envelope_ratio (1) x eps_rel (1e-3) x the diagonal of the
    // box, grown over the features (x from 0 to 2.3).
    const double radius = 1e-3 * Eigen::Vector3d(2.3, 2, 2).norm();
    const auto features = t.feature_points();
    CHECK(features.size() == n_features);
    const auto& in = t.map().input_vertices();
    for (const auto& f : features) {
        if (f.output_vertex == tracking::kNoId) continue;
        REQUIRE(f.output_vertex < uint64_t(mesh.V.rows()));
        m.features_kept += (V(f.output_vertex) - in[f.input_vertex]).norm() <= radius;
    }
    return m;
}

} // namespace

TEST_CASE("tetwild-curve-history-leaves-the-output-alone", "[tetwild][tracking]" + release_only)
{
    // Recording never changes the meshing: the same config gives the same mesh recorded or not.
    const ComponentResult recorded = run_curves({});
    const ComponentResult plain = run_curves({}, false);
    CHECK_FALSE(plain.second.contains("curve_history"));
    CHECK_FALSE(recorded.second.contains("history")); // its own key, without history_output
    const MeshData& a = recorded.first.at("mesh");
    const MeshData& b = plain.first.at("mesh");
    CHECK(a.V == b.V);
    CHECK(a.cell(CellType::Tetrahedron).connectivity == b.cell(CellType::Tetrahedron).connectivity);
}

TEST_CASE("tetwild-curve-history-preserve-topology", "[tetwild][tracking]" + release_only)
{
    // Under preserve_topology the map is a bijection: every point maps there and back, no
    // warning, and every feature point ends within its anchor's radius.
    const ComponentResult r = run_curves({{"preserve_topology", true}});
    const CurveMapped m = check_curve_history(r);
    CHECK(m.lost == 0);
    CHECK(m.round_trip < 1e-8);
    CHECK(m.features_kept == n_curve_features);
    CHECK(tracking::CurveTracking(curve_history_of(r)).map().build_warnings().total() == 0);
}

TEST_CASE("tetwild-curve-history-default-topology", "[tetwild][tracking]" + release_only)
{
    // Without preserve_topology the collapses may weld curves that pass within the tube: a
    // fourth curve runs along the chain, 1e-3 off it, closer than eps (3.5e-3). The welds are
    // warnings of the map, which still ends at the mesh and maps every point. Under
    // preserve_topology the substructure link condition refuses them -- for these curves off the
    // surface only since 2026-11 -- and the map is a bijection.
    for (const bool preserve_topology : {false, true}) {
        CAPTURE(preserve_topology);
        const ComponentResult r =
            run_curves({{"preserve_topology", preserve_topology}}, true, 1e-3);
        const CurveMapped m = check_curve_history(r, n_curve_segments + 4, n_curve_features + 2);
        CHECK(m.lost == 0);
        const tracking::CurveTracking t(curve_history_of(r));
        const auto& w = t.map().build_warnings();
        INFO(m.exact << " of " << m.points << " exact, worst " << m.round_trip);
        if (preserve_topology) {
            CHECK(w.topology == 0);
            CHECK(m.round_trip < 1e-8);
        } else {
            CHECK(w.topology > 0);
            CHECK(m.exact < m.points);
        }
    }
}

TEST_CASE("tetwild-curve-history-filters", "[tetwild][tracking]" + release_only)
{
    // A filter removes the tets outside the cube after the optimization, and with them the curve
    // past x = 2: the history records that as segments replaced by nothing (resync,
    // replace_faces), and ends at the filtered mesh. Points that were on it have no image, nor
    // its two ends; every other point maps there and back.
    for (const std::string filter : {"input", "flood"}) {
        INFO("filter " << filter);
        const ComponentResult r = run_curves({{"filter", filter}, {"preserve_topology", true}});
        const CurveMapped m = check_curve_history(r);
        CHECK(m.lost > 0);
        CHECK(m.lost < m.points);
        CHECK(m.round_trip < 1e-8);
        CHECK(m.features_kept == n_curve_features - 2);
        const tracking::CurveTracking t(curve_history_of(r));
        CHECK(t.map().build_warnings().replaced > 0);
        CHECK(t.map().build_warnings().topology == 0);
    }
}

TEST_CASE("tetwild-curve-history-tagged-route", "[tetwild][tracking]" + release_only)
{
    // tetwild's own output taken back in, its edge block the feature curves: the history starts
    // at them as received -- a segment per tracked edge of that mesh, at its positions -- and
    // ends at the output's, as the surface history of a tagged mesh input does.
    const ComponentResult first = run_curves({{"preserve_topology", true}}, false);
    const MeshData& received = first.first.at("mesh");
    const ComponentResult r = tetwild::tetwild(
        {{"input", {"mesh"}},
         {"curve_history_output", "record"},
         {"preserve_topology", true},
         {"max_iterations", 2}},
        {{"mesh", received}});
    const history::CurveHistory h = curve_history_of(r);
    const tracking::CurveTracking t(h);
    const auto& E = received.cell(CellType::Edge).connectivity;
    REQUIRE(t.input_segments().size() == size_t(E.rows()));
    std::set<Key> given, started;
    const auto key2 = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return key_of(a, b, b);
    };
    for (Eigen::Index i = 0; i < E.rows(); ++i) {
        given.insert(
            key2(received.V.row(E(i, 0)).transpose(), received.V.row(E(i, 1)).transpose()));
    }
    const auto& IV = t.map().input_vertices();
    for (const auto& s : t.input_segments()) started.insert(key2(IV[s[0]], IV[s[1]]));
    CHECK(started == given);
    CHECK(t.map().build_warnings().total() == 0);
    std::set<std::array<uint64_t, 2>> tracked, block;
    for (const auto& e : t.output_segments()) {
        tracked.insert({{std::min(e[0], e[1]), std::max(e[0], e[1])}});
    }
    const auto& E2 = r.first.at("mesh").cell(CellType::Edge).connectivity;
    for (Eigen::Index i = 0; i < E2.rows(); ++i) {
        const uint64_t x = uint64_t(E2(i, 0)), y = uint64_t(E2(i, 1));
        block.insert({{std::min(x, y), std::max(x, y)}});
    }
    CHECK(tracked == block);
}

TEST_CASE("tetwild-curve-history-file-and-memory-routes", "[tetwild][tracking]" + release_only)
{
    // The same curves through files and in memory record the same curve history: the same first
    // state, the same edits, the same output ids. Only the parameters differ, the route among
    // them. Beside history_output, each in its own file: the surface history is unchanged by it.
    const fs::path dir = fs::current_path() / "tetwild_curve_history_file_and_memory";
    fs::remove_all(dir);
    fs::create_directories(dir);
    for (const auto& [name, m] : curve_scene()) write_obj(dir / (name + ".obj"), m);
    const nlohmann::json options = {{"preserve_topology", true}, {"max_iterations", 3}};
    nlohmann::json file = options;
    file.update({
        {"application", "tetwild"},
        {"input",
         {(dir / "cube.obj").string(),
          (dir / "chain.obj").string(),
          (dir / "rim.obj").string(),
          (dir / "outside.obj").string(),
          (dir / "points.obj").string()}},
        {"output", (dir / "out").string()},
        {"history_output", (dir / "out.wmtkhist").string()},
        {"curve_history_output", (dir / "out_curves.wmtkhist").string()},
        {"write_vtu", false},
    });
    tetwild::tetwild(file);
    const auto a = history::CurveHistory::load((dir / "out_curves.wmtkhist").string());
    nlohmann::json memory = options;
    memory["history_output"] = "record";
    const ComponentResult r = run_curves(memory);
    REQUIRE(r.second.at("history").is_binary());
    const history::CurveHistory b = curve_history_of(r);
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
    // The surface history is the same with the curves recorded or not.
    const auto surface = history::SurfaceHistory::load((dir / "out.wmtkhist").string());
    nlohmann::json without = file;
    without.erase("curve_history_output");
    without["history_output"] = (dir / "plain.wmtkhist").string();
    without["output"] = (dir / "plain").string();
    tetwild::tetwild(without);
    CHECK_FALSE(fs::exists(dir / "plain_curves.wmtkhist"));
    const auto plain = history::SurfaceHistory::load((dir / "plain.wmtkhist").string());
    CHECK(surface.edits.size() == plain.edits.size());
    CHECK(surface.output_vertex_ids == plain.output_vertex_ids);
    // The output the file route wrote is the mesh the memory route returned.
    CHECK(io::read_mesh(dir / "out_final.msh").V == r.first.at("mesh").V);
    fs::remove_all(dir);
}
