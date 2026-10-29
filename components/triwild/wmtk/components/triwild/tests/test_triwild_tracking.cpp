// How to use triwild's tracking file: map your own points, or attributes, between the input
// curves and the tracked curves of the output triangle mesh. A point on the input is a segment
// of the input curve files and barycentric coordinates; a point on the output is named by two
// vertices of the mesh -- the ends of a tracked edge -- since the tracked curves are no element
// of the output file. The input's feature points (curve ends, junctions, input points) come with
// the output vertex each ended at.

#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/components/triwild/triwild.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <set>
#include <sstream>

namespace fs = std::filesystem;
using namespace wmtk;
using namespace wmtk::components;
using tracking::CurveTracking;
using tracking::FeaturePoint;
using tracking::InputCurvePoint;
using tracking::kNoId;
using tracking::OutputCurvePoint;

TEST_CASE("tracking-example-triwild", "[triwild][tracking]")
{
    const fs::path dir = fs::current_path() / "tracking_example_triwild";
    fs::create_directories(dir);

    // The input: a circle, an open curve crossing it twice, and two free points. Their
    // vertices, as the files hold them: the curve files' in order, then the point file's.
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::array<size_t, 2>> segments; // of the curve files, in order
    {
        std::ofstream circle(dir / "circle.obj"), open(dir / "open.obj"),
            points(dir / "points.obj");
        circle.precision(17);
        open.precision(17);
        const int n = 300;
        for (int i = 0; i < n; ++i) {
            const double a = 2 * M_PI * i / n;
            vertices.push_back(Eigen::Vector3d(std::cos(a), std::sin(a), 0));
            circle << "v " << vertices.back().x() << " " << vertices.back().y() << " 0\n";
            segments.push_back({{size_t(i), size_t((i + 1) % n)}});
        }
        circle << "l";
        for (int i = 0; i <= n; ++i) circle << " " << i % n + 1;
        circle << "\n";
        const int m = 100;
        for (int i = 0; i <= m; ++i) {
            const double x = -1.6 + 3.2 * i / m;
            vertices.push_back(Eigen::Vector3d(x, 0.3 * std::sin(6 * x) + 0.1, 0));
            open << "v " << vertices.back().x() << " " << vertices.back().y() << " 0\n";
            if (i > 0) segments.push_back({{size_t(n + i - 1), size_t(n + i)}});
        }
        open << "l";
        for (int i = 0; i <= m; ++i) open << " " << i + 1;
        open << "\n";
        for (const Eigen::Vector3d& p :
             {Eigen::Vector3d(0.25, -0.5, 0), Eigen::Vector3d(-0.7, 0.9, 0)}) {
            vertices.push_back(p);
            points << "v " << p.x() << " " << p.y() << " 0\n";
        }
    }
    const fs::path output = dir / "curves";
    const fs::path tracking_file = dir / "curves.wmtkhist";

    // 1. Run triwild with tracking on: history_output names the tracking file. The mesh is
    // written to <output>.msh, whose vertices the tracking names. preserve_topology keeps the
    // map a bijection; with history_output set, triwild anchors the junctions too
    // (allow_junction_cleanup), so the map is not stretched by one sliding along a curve.
    triwild::triwild({
        {"application", "triwild"},
        {"input", {(dir / "circle.obj").string(), (dir / "open.obj").string()}},
        {"input_points", {(dir / "points.obj").string()}},
        {"output", output.string()},
        {"history_output", tracking_file.string()},
        {"preserve_topology", true},
        {"length_rel", 0.02},
        {"max_iterations", 10},
        {"write_vtu", false},
    });
    std::vector<Eigen::Vector3d> mesh_vertices;
    std::set<std::array<uint64_t, 2>> mesh_edges;
    {
        const MeshData msh = io::read_mesh(output.string() + ".msh");
        for (Eigen::Index i = 0; i < msh.V.rows(); ++i) {
            mesh_vertices.push_back(msh.V.row(i).transpose());
        }
        const auto& F = msh.cell(CellType::Triangle).connectivity;
        for (Eigen::Index f = 0; f < F.rows(); ++f) {
            for (int k = 0; k < 3; ++k) {
                const uint64_t x = uint64_t(F(f, k)), y = uint64_t(F(f, (k + 1) % 3));
                mesh_edges.insert({{std::min(x, y), std::max(x, y)}});
            }
        }
    }
    const auto on_input = [&](const InputCurvePoint& p) {
        const auto& s = segments[p.segment];
        return Eigen::Vector3d(p.bary[0] * vertices[s[0]] + p.bary[1] * vertices[s[1]]);
    };
    const auto on_output = [&](const OutputCurvePoint& p) {
        return Eigen::Vector3d(
            p.bary[0] * mesh_vertices[p.vertices[0]] + p.bary[1] * mesh_vertices[p.vertices[1]]);
    };

    // 2. Load it. The output's tracked curves are edges of the mesh, by its vertex ids.
    const CurveTracking tracking = CurveTracking::load(tracking_file.string());
    REQUIRE(tracking.output_by_vertices()); // a triangle mesh output: points are vertex pairs
    CHECK(tracking.map().build_warnings().total() == 0);
    REQUIRE(tracking.input_segments().size() == segments.size());
    double length = 0;
    for (const auto& e : tracking.output_segments()) {
        for (const uint64_t v : e) REQUIRE(v < mesh_vertices.size());
        CHECK(mesh_edges.count({{std::min(e[0], e[1]), std::max(e[0], e[1])}}) == 1);
        length += (mesh_vertices[e[0]] - mesh_vertices[e[1]]).norm();
    }
    const double mean_length = length / double(tracking.output_segments().size());

    // 3. Map points on the input -- a segment of the input files and barycentric coordinates --
    // to the output, and back. (The coordinates are the generator's own numbers, the same
    // everywhere; std::uniform_real_distribution's differ between standard libraries.)
    std::mt19937 rng(3);
    std::vector<InputCurvePoint> pts;
    for (int i = 0; i < 500; ++i) {
        const double t = double(rng()) / 4294967296.0;
        pts.push_back({uint64_t(rng() % segments.size()), Eigen::Vector2d(1 - t, t)});
    }
    const std::vector<OutputCurvePoint> there = tracking.to_output(pts);
    const std::vector<InputCurvePoint> back = tracking.to_input(there);
    double round_trip = 0, moved = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].vertices[0] != kNoId);
        for (const uint64_t v : there[i].vertices) REQUIRE(v < mesh_vertices.size());
        REQUIRE(back[i].segment == pts[i].segment);
        round_trip = std::max(round_trip, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
        moved = std::max(moved, (on_output(there[i]) - on_input(pts[i])).norm());
    }
    CHECK(round_trip < 1e-8);
    CHECK(moved < mean_length);

    // 4. The feature points: the open curve's two ends and the two free points, each at the
    // output vertex it ended at -- within the envelope, eps_rel times the input's diagonal.
    const std::vector<FeaturePoint> features = tracking.feature_points();
    CHECK(features.size() == 4);
    for (const FeaturePoint& f : features) {
        REQUIRE(f.input_vertex < vertices.size());
        REQUIRE(f.output_vertex < mesh_vertices.size());
        CHECK((mesh_vertices[f.output_vertex] - vertices[f.input_vertex]).norm() < 1e-3 * 4);
    }
    // A point at an end of the open curve maps onto the end's output vertex.
    const InputCurvePoint end{segments.size() - 1, Eigen::Vector2d(0, 1)};
    const OutputCurvePoint at_end = tracking.to_output({end})[0];
    const auto f_end = std::find_if(features.begin(), features.end(), [&](const FeaturePoint& f) {
        return f.input_vertex == segments.back()[1];
    });
    REQUIRE(f_end != features.end());
    CHECK((on_output(at_end) - mesh_vertices[f_end->output_vertex]).norm() < 1e-12);

    // 5. The same from the command line, as the wmtk binary runs it: points on the output, one
    // `v0 v1 b0 b1` per line (the ends in either order), mapped back to the input.
    const fs::path points = dir / "points_on_output.txt";
    {
        std::ofstream o(points);
        o.precision(17);
        for (const OutputCurvePoint& p : there) {
            // The ends listed backwards, to show the order is free.
            o << p.vertices[1] << " " << p.vertices[0] << " " << p.bary[1] << " " << p.bary[0]
              << "\n";
        }
    }
    const fs::path mapped = dir / "points_on_input.txt";
    tracking::tracking({
        {"application", "tracking"},
        {"history", tracking_file.string()},
        {"operation", "map"},
        {"points", points.string()},
        {"output_points", mapped.string()},
        {"direction", "to_input"},
    });
    std::ifstream in(mapped);
    size_t i = 0, same = 0;
    for (std::string line; std::getline(in, line); ++i) {
        std::istringstream ss(line);
        int64_t segment;
        Eigen::Vector2d b;
        ss >> segment >> b[0] >> b[1];
        REQUIRE(i < pts.size());
        same += uint64_t(segment) == pts[i].segment && (b - pts[i].bary).norm() < 1e-8;
    }
    CHECK(i == pts.size());
    CHECK(same == pts.size());
}

TEST_CASE("tracking-memory-route-triwild", "[triwild][tracking]")
{
    // The memory route: triwild records its history from the curves it is given and returns it
    // in its report, as bytes, writing no file, and the tracking application's info reports on
    // it, the report returned rather than logged only. The input is the example's above, as
    // meshes.
    const fs::path dir = fs::current_path() / "tracking_memory_route_triwild";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const fs::path tracking_file = dir / "curves.wmtkhist";
    const int n = 300, m = 100;
    MeshData circle, open, points;
    circle.V.resize(n, 3);
    auto& C = circle.cells[CellType::Edge].connectivity;
    C.resize(n, 2);
    for (int i = 0; i < n; ++i) {
        const double a = 2 * M_PI * i / n;
        circle.V.row(i) << std::cos(a), std::sin(a), 0;
        C.row(i) << i, (i + 1) % n;
    }
    open.V.resize(m + 1, 3);
    auto& O = open.cells[CellType::Edge].connectivity;
    O.resize(m, 2);
    for (int i = 0; i <= m; ++i) {
        const double x = -1.6 + 3.2 * i / m;
        open.V.row(i) << x, 0.3 * std::sin(6 * x) + 0.1, 0;
        if (i > 0) O.row(i - 1) << i - 1, i;
    }
    points.V.resize(2, 3);
    points.V << 0.25, -0.5, 0, -0.7, 0.9, 0;
    points.cells[CellType::Point].connectivity.resize(2, 1);
    points.cells[CellType::Point].connectivity << 0, 1;
    const MeshMap meshes{{"circle", circle}, {"open", open}, {"points", points}};
    const ComponentResult result = triwild::triwild(
        {{"input", {"circle", "open"}},
         {"input_points", {"points"}},
         {"history_output", tracking_file.string()},
         {"preserve_topology", true},
         {"length_rel", 0.02},
         {"max_iterations", 10}},
        meshes);
    CHECK(fs::is_empty(dir));
    REQUIRE(result.second.contains("history"));
    const nlohmann::json bytes = result.second.at("history");
    REQUIRE(bytes.is_binary());
    CHECK(history::face_size(bytes.get_binary()) == 2);
    CHECK(
        nlohmann::json::parse(history::CurveHistory::from_bytes(bytes.get_binary()).metadata)
            .at("route") == "memory");

    // The history starts at the curves given, segment for segment, and its tracked edges are edges
    // of the mesh returned, by its vertex rows. The bytes are the file the file route writes.
    history::write_history(tracking_file, bytes.get_binary());
    const CurveTracking tracking = CurveTracking::load(tracking_file.string());
    REQUIRE(tracking.output_by_vertices());
    CHECK(tracking.input_segments().size() == size_t(n + m));
    const MeshData& mesh = result.first.at("mesh");
    std::set<std::array<uint64_t, 2>> mesh_edges;
    const auto& F = mesh.cell(CellType::Triangle).connectivity;
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            const uint64_t x = uint64_t(F(f, k)), y = uint64_t(F(f, (k + 1) % 3));
            mesh_edges.insert({{std::min(x, y), std::max(x, y)}});
        }
    }
    REQUIRE(!tracking.output_segments().empty());
    for (const auto& e : tracking.output_segments()) {
        CHECK(mesh_edges.count({{std::min(e[0], e[1]), std::max(e[0], e[1])}}) == 1);
    }
    // The open curve's two ends and the two free points.
    CHECK(tracking.feature_points().size() == 4);

    // tracking takes the history as bytes on the memory route.
    const auto [none, report] = tracking::tracking({{"history", bytes}, {"operation", "info"}}, {});
    CHECK(none.empty());
    CHECK(report.at("edits").get<size_t>() > 0);
    CHECK(report.at("build_warnings").at("total") == 0);
    CHECK(report.at("feature_points") == 4);
    CHECK(report.at("feature_points_at_output_vertex") == 4);
    CHECK(report.at("midpoints") == tracking.output_segments().size());
    CHECK(report.at("round_trip_lost") == 0);
    CHECK(report.at("round_trip_max_displacement").get<double>() < 1e-8);

    // features: the feature points in the report, each an input vertex and the output vertex
    // it became, as CurveTracking has them; nothing written.
    const auto [no_meshes, features] =
        tracking::tracking({{"history", bytes}, {"operation", "features"}}, {});
    CHECK(no_meshes.empty());
    const auto expected = tracking.feature_points();
    REQUIRE(features.at("feature_points").size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        CHECK(features["feature_points"][i][0].get<int64_t>() == int64_t(expected[i].input_vertex));
        CHECK(
            features["feature_points"][i][1].get<int64_t>() == int64_t(expected[i].output_vertex));
    }
    CHECK(features.at("feature_points_at_output_vertex") == 4);
    // The one file in the directory is the history written above.
    CHECK(std::distance(fs::directory_iterator(dir), fs::directory_iterator()) == 1);
    // map reads and writes files: refused on the memory route.
    CHECK_THROWS_AS(
        tracking::tracking({{"history", bytes}, {"operation", "map"}}, {}),
        std::runtime_error);
}
