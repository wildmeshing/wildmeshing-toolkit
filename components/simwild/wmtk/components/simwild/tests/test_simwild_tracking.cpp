// How to use simwild's tracking file: map your own points, or attributes, between the input
// surfaces and the tracked surface of the output tet mesh -- the interfaces between its tags. A
// point on the output is named by three vertices of the tet mesh and barycentric coordinates.

#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/history/SurfaceHistory.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <igl/readOBJ.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

namespace fs = std::filesystem;
using namespace wmtk;
using namespace wmtk::components;
using tracking::InputPoint;
using tracking::kNoId;
using tracking::OutputPoint;
using tracking::Tracking;

// A whole run, too slow for the Debug CI jobs: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

TEST_CASE("tracking-example-simwild", "[simwild][tracking]" + release_only)
{
    const fs::path dir = fs::current_path() / "tracking_example_simwild";
    fs::create_directories(dir);
    const fs::path sphere = fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj";
    const fs::path output = dir / "spheres";
    const fs::path tracking_file = dir / "spheres.wmtkhist";

    // 1. Run simwild on two input surfaces with tracking on (history_output). The tet mesh is
    // written to <output>.msh, whose vertices the tracking names.
    simwild::simwild({
        {"application", "simwild"},
        {"operation", "remeshing"},
        {"input", {sphere.string(), sphere.string()}},
        {"input_transform",
         {nlohmann::json::array(), {{1, 0, 0, 0.3}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}}},
        {"output", output.string()},
        {"history_output", tracking_file.string()},
        {"eps_rel", 1e-2},
        {"stop_energy", 10},
        {"max_iterations", 3},
    });
    std::vector<Eigen::Vector3d> tet_vertices;
    {
        const MeshData msh = io::read_mesh(output.string() + ".msh");
        for (Eigen::Index i = 0; i < msh.V.rows(); ++i) {
            tet_vertices.push_back(msh.V.row(i).transpose());
        }
    }
    // The input: the triangles of both files, in order; the second is moved by its transform.
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    REQUIRE(igl::readOBJ(sphere.string(), V, F));
    const auto on_input = [&](const InputPoint& p) {
        const uint64_t f = p.face % F.rows();
        Eigen::Vector3d q =
            p.bary[0] * V.row(F(f, 0)) + p.bary[1] * V.row(F(f, 1)) + p.bary[2] * V.row(F(f, 2));
        if (p.face >= uint64_t(F.rows())) q.x() += 0.3;
        return q;
    };

    // 2. Map points on the input -- a triangle of the input files and barycentric coordinates
    // -- to the output, and back. A part of an input surface inside the other one is no
    // interface of the output, and comes back with no image.
    const Tracking tracking = Tracking::load(tracking_file.string());
    REQUIRE(tracking.output_by_vertices());
    REQUIRE(tracking.input_faces().size() == size_t(2 * F.rows()));
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u(0, 1);
    std::vector<InputPoint> pts;
    for (int i = 0; i < 500; ++i) {
        double a = u(rng), b = u(rng);
        if (a + b > 1) {
            a = 1 - a;
            b = 1 - b;
        }
        pts.push_back({uint64_t(rng() % (2 * F.rows())), Eigen::Vector3d(1 - a - b, a, b)});
    }
    const std::vector<OutputPoint> there = tracking.to_output(pts);
    const std::vector<InputPoint> back = tracking.to_input(there);
    size_t mapped = 0;
    double round_trip = 0, moved = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (there[i].vertices[0] == kNoId) continue;
        ++mapped;
        REQUIRE(back[i].face == pts[i].face);
        round_trip = std::max(round_trip, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
        const Eigen::Vector3d q = there[i].bary[0] * tet_vertices[there[i].vertices[0]] +
                                  there[i].bary[1] * tet_vertices[there[i].vertices[1]] +
                                  there[i].bary[2] * tet_vertices[there[i].vertices[2]];
        moved = std::max(moved, (q - on_input(pts[i])).norm());
    }
    CHECK(mapped > pts.size() / 2);
    CHECK(round_trip < 1e-8);
    CHECK(moved < 0.2);

    // 3. Every point of the output has a preimage: each corner of a tracked face maps to the input.
    std::vector<OutputPoint> corners;
    for (const auto& t : tracking.output_triangles()) {
        for (int k = 0; k < 3; ++k) corners.push_back({kNoId, t, Eigen::Vector3d::Unit(k)});
    }
    size_t lost = 0;
    for (const InputPoint& s : tracking.to_input(corners)) lost += s.face == kNoId;
    CHECK(lost == 0);
}

TEST_CASE("tracking-example-simwild-tet-input", "[simwild][tracking]")
{
    // A tet mesh as input: the history starts at its tracked surface, the interfaces between its
    // tags, which no input file holds as triangles. A point on it is named as one on the output
    // is: three vertices of the mesh the history starts from -- written next to the output as
    // <output>_history_input.msh -- and barycentric coordinates.
    const fs::path dir = fs::current_path() / "tracking_example_simwild_tet_input";
    fs::create_directories(dir);
    const fs::path output = dir / "double_sphere";
    const fs::path tracking_file = dir / "double_sphere.wmtkhist";
    simwild::simwild({
        {"application", "simwild"},
        {"operation", "remeshing"},
        {"input", {(fs::path(WMTK_DATA_DIR) / "models" / "double_sphere.msh").string()}},
        {"output", output.string()},
        {"history_output", tracking_file.string()},
        {"max_iterations", 2},
    });
    const MeshData start = io::read_mesh(output.string() + "_history_input.msh");
    const MeshData end = io::read_mesh(output.string() + ".msh");
    const auto at =
        [](const MeshData& m, const std::array<uint64_t, 3>& v, const Eigen::Vector3d& b) {
            Eigen::Vector3d q = Eigen::Vector3d::Zero();
            for (int k = 0; k < 3; ++k) q += b[k] * m.V.row(v[k]).transpose();
            return q;
        };

    const Tracking tracking = Tracking::load(tracking_file.string());
    REQUIRE(tracking.input_by_vertices());
    REQUIRE(tracking.output_by_vertices());
    // The input's tracked faces by the start mesh's vertices, where the history has them.
    const auto triangles = tracking.input_triangles();
    REQUIRE(!triangles.empty());
    for (size_t f = 0; f < triangles.size(); ++f) {
        for (int k = 0; k < 3; ++k) {
            REQUIRE(triangles[f][k] < uint64_t(start.V.rows()));
            const auto& p = tracking.map().input_vertices()[tracking.input_faces()[f][k]];
            REQUIRE(start.V.row(triangles[f][k]).transpose() == p);
        }
    }

    // Points on those faces, their corners listed in any order, to the output and back.
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(0, 1);
    std::vector<InputPoint> pts;
    for (int i = 0; i < 300; ++i) {
        double a = u(rng), b = u(rng);
        if (a + b > 1) {
            a = 1 - a;
            b = 1 - b;
        }
        InputPoint p;
        p.vertices = triangles[rng() % triangles.size()];
        p.bary = Eigen::Vector3d(1 - a - b, a, b);
        if (i % 2) {
            std::swap(p.vertices[0], p.vertices[2]);
            std::swap(p.bary[0], p.bary[2]);
        }
        pts.push_back(p);
    }
    tracking::MapWarnings warnings;
    const std::vector<OutputPoint> there = tracking.to_output(pts, &warnings);
    const std::vector<InputPoint> back = tracking.to_input(there, &warnings);
    CHECK(warnings.total() == 0);
    double round_trip = 0, moved = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].vertices[0] != kNoId);
        REQUIRE(back[i].vertices[0] != kNoId);
        round_trip = std::max(
            round_trip,
            (at(start, back[i].vertices, back[i].bary) - at(start, pts[i].vertices, pts[i].bary))
                .norm());
        moved = std::max(
            moved,
            (at(end, there[i].vertices, there[i].bary) - at(start, pts[i].vertices, pts[i].bary))
                .norm());
    }
    CHECK(round_trip < 1e-8);
    CHECK(moved < 0.2);
    // A triplet that is no tracked face has no image.
    InputPoint none;
    none.vertices = {{triangles[0][0], triangles[0][1], uint64_t(start.V.rows())}};
    CHECK(tracking.to_output({none})[0].vertices[0] == kNoId);

    // The same from the command line: `v0 v1 v2 b0 b1 b2` on either side.
    const fs::path points = dir / "points.txt";
    {
        std::ofstream o(points);
        o.precision(17);
        for (const InputPoint& p : pts) {
            for (const uint64_t v : p.vertices) o << v << " ";
            o << p.bary[0] << " " << p.bary[1] << " " << p.bary[2] << "\n";
        }
    }
    const fs::path mapped = dir / "points_on_output.txt";
    tracking::tracking({
        {"application", "tracking"},
        {"history", tracking_file.string()},
        {"operation", "map"},
        {"points", points.string()},
        {"output_points", mapped.string()},
        {"direction", "to_output"},
    });
    std::ifstream in(mapped);
    size_t i = 0, same = 0;
    for (std::string line; std::getline(in, line); ++i) {
        std::istringstream ss(line);
        std::array<uint64_t, 3> v;
        Eigen::Vector3d b;
        ss >> v[0] >> v[1] >> v[2] >> b[0] >> b[1] >> b[2];
        REQUIRE(i < there.size());
        same += v == there[i].vertices && (b - there[i].bary).norm() < 1e-12;
    }
    CHECK(i == pts.size());
    CHECK(same == pts.size());
}

TEST_CASE("tracking-simwild-memory-route", "[simwild][tracking]")
{
    // The memory route writes no file: the history comes back in the report, as the bytes of
    // the file, and the mesh it starts from -- <output>_history_input.msh on the file route --
    // as the output mesh history_input, whose vertices name the points on the input.
    const MeshMap meshes{
        {"in", io::read_mesh(fs::path(WMTK_DATA_DIR) / "models" / "double_sphere.msh")}};
    const auto [outputs, report] = simwild::simwild(
        {{"operation", "remeshing"},
         {"input", "in"},
         {"history_output", "recorded"},
         {"max_iterations", 1}},
        meshes);
    CHECK(!fs::exists("recorded"));
    REQUIRE(report.at("history").is_binary());
    REQUIRE(outputs.count("history_input"));
    REQUIRE(outputs.count("mesh"));
    const Tracking tracking(history::SurfaceHistory::from_bytes(report.at("history").get_binary()));
    REQUIRE(tracking.input_by_vertices());
    REQUIRE(tracking.output_by_vertices());
    const MeshData& start = outputs.at("history_input");
    const MeshData& end = outputs.at("mesh");
    CHECK(start.cell(CellType::Tetrahedron).attributes.count("tags"));
    const auto& faces = tracking.input_faces();
    const auto triangles = tracking.input_triangles();
    for (size_t f = 0; f < triangles.size(); ++f) {
        for (int k = 0; k < 3; ++k) {
            REQUIRE(triangles[f][k] < uint64_t(start.V.rows()));
            REQUIRE(
                start.V.row(triangles[f][k]).transpose() ==
                tracking.map().input_vertices()[faces[f][k]]);
        }
    }
    for (const auto& t : tracking.output_triangles()) {
        for (const uint64_t v : t) REQUIRE(v < uint64_t(end.V.rows()));
    }
}

TEST_CASE("tracking-simwild-2d-refuses-history", "[simwild][tracking]")
{
    // A 2D run has no surface: history_output is refused rather than ignored.
    MeshData square;
    square.V.resize(4, 3);
    square.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0;
    auto& t = square.cells[CellType::Triangle];
    t.connectivity.resize(2, 3);
    t.connectivity << 0, 1, 2, 0, 2, 3;
    t.attributes["tags"].values = DataMatrix<uint8_t>(DataMatrix<uint8_t>::Ones(2, 1));
    t.attributes["tags"].labels = {"tag_0"};
    CHECK_THROWS_WITH(
        simwild::simwild(
            {{"input", "square"}, {"history_output", "recorded"}, {"max_iterations", 0}},
            {{"square", square}}),
        Catch::Matchers::ContainsSubstring("3D only"));
    CHECK(!fs::exists("recorded"));
}
