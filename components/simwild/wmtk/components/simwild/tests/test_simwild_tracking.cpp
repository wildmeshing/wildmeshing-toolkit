// How to use simwild's tracking file: map your own points, or attributes, between the input
// surfaces and the tracked surface of the output tet mesh -- the interfaces between its tags. A
// point on the output is named by three vertices of the tet mesh and barycentric coordinates.

#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/components/tracking/VolumeMap.hpp>
#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/components/tracking/volume_io.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/history/VolumeHistory.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>

#include <igl/readOBJ.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <set>
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

TEST_CASE("tracking-example-simwild-tet-input", "[simwild][tracking]" + release_only)
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

TEST_CASE("tracking-simwild-memory-route", "[simwild][tracking]" + release_only)
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

TEST_CASE("tracking-simwild-memory-route-volume", "[simwild][tracking]" + release_only)
{
    // With volume_history_output, the memory route returns the volume history in the report too,
    // as the bytes of its file, and writes none: with the surface history's bytes and the two
    // meshes it returns -- history_input, where both histories start, and mesh -- they are a
    // VolumeTracking, of the tets of those meshes.
    const MeshMap meshes{
        {"in", io::read_mesh(fs::path(WMTK_DATA_DIR) / "models" / "double_sphere.msh")}};
    const auto [outputs, report] = simwild::simwild(
        {{"operation", "remeshing"},
         {"input", "in"},
         {"history_output", "recorded"},
         {"volume_history_output", "recorded_volume"},
         {"max_iterations", 1}},
        meshes);
    CHECK(!fs::exists("recorded"));
    CHECK(!fs::exists("recorded_volume"));
    REQUIRE(report.at("history").is_binary());
    REQUIRE(report.at("volume_history").is_binary());
    const auto& bytes = report.at("volume_history").get_binary();
    REQUIRE(history::is_volume_history(bytes));
    const auto surface = history::SurfaceHistory::from_bytes(report.at("history").get_binary());
    auto volume = history::VolumeHistory::from_bytes(bytes);
    REQUIRE(!volume.edits.empty());
    CHECK(volume.surface == history::surface_key(surface));
    const tracking::VolumeTracking tracking(
        surface,
        std::move(volume),
        outputs.at("history_input"),
        outputs.at("mesh"));
    const auto& in = tracking.input_mesh();
    const auto& out = tracking.output_mesh();
    CHECK(
        in.tets.size() ==
        size_t(outputs.at("history_input").cell(CellType::Tetrahedron).connectivity.rows()));
    CHECK(
        out.tets.size() ==
        size_t(outputs.at("mesh").cell(CellType::Tetrahedron).connectivity.rows()));
    const auto position = [](const tracking::TetMeshData& m, const tracking::TetPoint& p) {
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) x += p.bary[k] * m.vertices[m.tets[p.tet][k]];
        return x;
    };
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u(0.05, 1);
    std::vector<tracking::TetPoint> pts;
    for (int i = 0; i < 300; ++i) {
        tracking::TetPoint p{rng() % in.tets.size()};
        p.bary = Eigen::Vector4d(u(rng), u(rng), u(rng), u(rng));
        p.bary /= p.bary.sum();
        pts.push_back(p);
    }
    const auto there = tracking.to_output(pts);
    const auto back = tracking.to_input(there);
    CHECK(tracking.map().stats().failed == 0);
    double round_trip = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].tet != tracking::kNoId);
        CHECK(out.group_sets[out.groups[there[i].tet]] == in.group_sets[in.groups[pts[i].tet]]);
        REQUIRE(back[i].tet != tracking::kNoId);
        round_trip = std::max(round_trip, (position(in, back[i]) - position(in, pts[i])).norm());
    }
    CHECK(round_trip < 1e-6);
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

namespace {

/// How many of `pts` (at most the first 500 whose tets are not slivers) the map moves, and at
/// how many of those it keeps orientation: det J > 0, J by finite differences of
/// map_positions(), each of the four points in the region of the tet it is in (a step can
/// cross into another). In a sliver -- the insertion leaves some flat ones along the curves
/// where surfaces meet -- a step leaves the tet at once, and J says nothing of it.
std::pair<size_t, size_t> orientation_kept(
    const wmtk::components::tracking::VolumeTracking& tracking,
    const std::vector<wmtk::components::tracking::TetPoint>& pts)
{
    namespace tr = wmtk::components::tracking;
    const tr::TetMeshData& in = tracking.input_mesh();
    Eigen::Vector3d lo = in.vertices[0], hi = lo;
    for (const auto& v : in.vertices) {
        lo = lo.cwiseMin(v);
        hi = hi.cwiseMax(v);
    }
    const double h = 1e-9 * (hi - lo).norm();
    std::vector<Eigen::Vector3d> xyz;
    for (size_t i = 0; i < pts.size() && xyz.size() < 4 * 500; ++i) {
        const auto& t = in.tets[pts[i].tet];
        const auto& V = in.vertices;
        double l2 = 0;
        for (int a = 0; a < 4; ++a) {
            for (int b = a + 1; b < 4; ++b) l2 += (V[t[a]] - V[t[b]]).squaredNorm() / 6;
        }
        const double volume =
            std::abs((V[t[1]] - V[t[0]]).dot((V[t[2]] - V[t[0]]).cross(V[t[3]] - V[t[0]])));
        if (volume < 1e-2 * std::pow(l2, 1.5)) continue; // a sliver
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) x += pts[i].bary[k] * V[t[k]];
        for (int k = 0; k < 4; ++k) {
            xyz.push_back(k == 0 ? x : Eigen::Vector3d(x + h * Eigen::Vector3d::Unit(k - 1)));
        }
    }
    const auto y = tracking.map_positions(xyz, true);
    size_t moved = 0, kept = 0;
    for (size_t i = 0; 4 * i < y.size(); ++i) {
        if (y[4 * i].position == xyz[4 * i]) continue;
        ++moved;
        Eigen::Matrix3d J;
        for (int k = 0; k < 3; ++k) J.col(k) = (y[4 * i + k + 1].position - y[4 * i].position) / h;
        kept += J.determinant() > 0;
    }
    return {moved, kept};
}

} // namespace

TEST_CASE("tracking-example-simwild-volume", "[simwild][tracking]" + release_only)
{
    const fs::path dir = fs::current_path() / "tracking_example_simwild_volume";
    fs::create_directories(dir);
    const fs::path sphere = fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj";
    const fs::path output = dir / "spheres";

    // 1. Run simwild with the surface history (history_output) and the volume history
    // (volume_history_output). The volume map starts at <output>_history_input.msh, the two
    // surfaces embedded and tagged, and ends at <output>.msh. The inputs' names are the ones a
    // region is hardest to find by: the same twice, and "ambient", which the files label
    // "ambient" and "ambient_" -- and the untagged tets then in no group.
    simwild::simwild({
        {"application", "simwild"},
        {"operation", "remeshing"},
        {"input", {sphere.string(), sphere.string()}},
        {"input_names", {"ambient", "ambient"}},
        {"input_transform",
         {nlohmann::json::array(), {{1, 0, 0, 0.3}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}}},
        {"output", output.string()},
        {"history_output", output.string() + ".wmtkhist"},
        {"volume_history_output", output.string() + ".wmtkvol"},
        {"eps_rel", 1e-2},
        {"stop_energy", 10},
        {"max_iterations", 3},
    });

    // 2. Load the map: both histories, and the two meshes, which name the points.
    const tracking::VolumeTracking tracking = tracking::VolumeTracking::load(
        output.string() + ".wmtkhist",
        output.string() + ".wmtkvol",
        output.string() + "_history_input.msh",
        output.string() + ".msh");
    const auto& in = tracking.input_mesh();
    const auto& out = tracking.output_mesh();
    // every tet's groups name its region
    for (const auto* m : {&in, &out}) {
        for (const auto& g : m->group_sets) CHECK(tracking.map().region_of_groups(g));
    }
    CHECK(
        std::count(in.group_sets.begin(), in.group_sets.end(), std::set<std::string>{"ambient_"}));

    // 3. Points of the input -- a tet of the file and barycentric coordinates -- in the
    // spheres' boxes, where the operations are: each lands in a tet of the output of its own
    // region (the tets of the same tags), and comes back.
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(0, 1);
    const auto position = [](const tracking::TetMeshData& m, const tracking::TetPoint& p) {
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) x += p.bary[k] * m.vertices[m.tets[p.tet][k]];
        return x;
    };
    std::vector<tracking::TetPoint> pts;
    while (pts.size() < 2000) {
        tracking::TetPoint p{rng() % in.tets.size()};
        const Eigen::Vector3d c = position(in, p);
        if (c.norm() > 1.5 && (c - Eigen::Vector3d(0.3, 0, 0)).norm() > 1.5) continue;
        p.bary = Eigen::Vector4d(u(rng), u(rng), u(rng), u(rng));
        p.bary /= p.bary.sum();
        pts.push_back(p);
    }
    const auto there = tracking.to_output(pts);
    const auto back = tracking.to_input(there);
    CHECK(tracking.map().stats().failed == 0);
    CHECK(tracking.map().stats().surface_not_kept == 0);
    CHECK(tracking.map().stats().region_changed == 0);
    size_t moved = 0;
    double round_trip = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].tet != tracking::kNoId);
        CHECK(out.group_sets[out.groups[there[i].tet]] == in.group_sets[in.groups[pts[i].tet]]);
        moved += (position(out, there[i]) - position(in, pts[i])).norm() > 0;
        REQUIRE(back[i].tet != tracking::kNoId);
        round_trip = std::max(round_trip, (position(in, back[i]) - position(in, pts[i])).norm());
    }
    CHECK(moved > pts.size() / 10);
    // Rounding, through local maps whose rays can graze a face of a sliver: more than the
    // surface's, far less than any map that is not a bijection would miss by (a tet, 1e-2 and up
    // here). The worst point leaves one operation's patch through a face 3e-6 as high as it is
    // long, at a grazing angle, which magnifies the rounding before it about 6e5 times: 3.5e-7
    // with Eigen 3.4, 1.3e-6 (Apple clang) and 1.9e-6 (GCC) with Eigen 5.0.1 on arm64.
    CHECK(round_trip < 1e-5);
    // A round trip the identity passes too: the points that move keep their orientation.
    const auto [moving, kept_orientation] = orientation_kept(tracking, pts);
    CHECK(moving > 0);
    CHECK(kept_orientation == moving);
}

namespace {

/// The cube [0,1]^3 as a triangle mesh.
MeshData unit_cube_surface()
{
    MatrixXd V(8, 3);
    V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1;
    MatrixXi F(12, 3);
    F << 0, 3, 2, 0, 2, 1, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5;
    return io::geometry_data(V, F, CellType::Triangle);
}

/// A bent chain of four segments inside the cube, and two vertices with no edge -- feature
/// points by the edge-file rule -- one inside the cube, one outside its bounding box.
MeshData chain_and_points()
{
    MatrixXd V(7, 3);
    V << 0.2, 0.5, 0.5, 0.35, 0.5, 0.5, 0.5, 0.5, 0.5, 0.65, 0.65, 0.5, 0.8, 0.8, 0.5, //
        0.3, 0.3, 0.7, 1.5, 0.5, 0.5;
    MatrixXi E(4, 2);
    E << 0, 1, 1, 2, 2, 3, 3, 4;
    return io::geometry_data(V, E, CellType::Edge);
}

} // namespace

TEST_CASE("tracking-simwild-curves", "[simwild][tracking]" + release_only)
{
    // simwild records the feature curves of a surface input as tetwild does (curve_history_output),
    // from the curve file as read to the output; on the memory route the history is the report's
    // curve_history. It ends at the output's feature curves -- its edge block, edges of its tets
    // -- and under preserve_topology the map is a bijection: every point on the input curves maps
    // there and back, and each feature point (the chain's two ends, the two points) ends within
    // the anchors' radius, eps, of where it was.
    const MeshData features = chain_and_points();
    const auto [outputs, report] = simwild::simwild(
        {{"input", {"cube", "features"}},
         {"curve_history_output", "recorded"},
         {"preserve_topology", true},
         {"max_iterations", 3},
         {"num_threads", 1}},
        {{"cube", unit_cube_surface()}, {"features", features}});
    CHECK(!fs::exists("recorded"));
    CHECK(!report.contains("history")); // its own key, without history_output
    REQUIRE(report.at("curve_history").is_binary());
    const tracking::CurveTracking t(
        history::CurveHistory::from_bytes(report.at("curve_history").get_binary()));
    REQUIRE(t.output_by_vertices());
    CHECK(t.input_segments().size() == 4);
    CHECK(t.map().build_warnings().total() == 0);
    const MeshData& mesh = outputs.at("mesh");
    std::set<std::array<uint64_t, 2>> tet_edges, tracked, block;
    const auto& T = mesh.cell(CellType::Tetrahedron).connectivity;
    for (Eigen::Index i = 0; i < T.rows(); ++i) {
        for (int a = 0; a < 4; ++a) {
            for (int b = a + 1; b < 4; ++b) {
                const uint64_t x = uint64_t(T(i, a)), y = uint64_t(T(i, b));
                tet_edges.insert({{std::min(x, y), std::max(x, y)}});
            }
        }
    }
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
    CHECK(!tracked.empty());
    CHECK(tracked == block);

    std::mt19937 rng(7);
    std::vector<tracking::InputCurvePoint> pts;
    for (int i = 0; i < 500; ++i) {
        const double x = double(rng()) / 4294967296.0;
        pts.push_back({uint64_t(rng() % 4), Eigen::Vector2d(1 - x, x)});
    }
    const auto there = t.to_output(pts);
    const auto back = t.to_input(there);
    double round_trip = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].vertices[0] != kNoId);
        REQUIRE(back[i].segment == pts[i].segment);
        round_trip = std::max(round_trip, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
    }
    CHECK(round_trip < 1e-8);

    const double eps = report.at("eps").get<double>();
    const auto points = t.feature_points();
    CHECK(points.size() == 4);
    for (const auto& f : points) {
        REQUIRE(f.output_vertex < uint64_t(mesh.V.rows()));
        const Eigen::Vector3d at = mesh.V.row(Eigen::Index(f.output_vertex)).transpose();
        CHECK((at - features.V.row(Eigen::Index(f.input_vertex)).transpose()).norm() <= eps);
    }
}

TEST_CASE("tracking-simwild-curves-refusals", "[simwild][tracking]")
{
    // The curve history starts at the feature curve files, which only a 3D surface input takes:
    // a tagged tet mesh and a 2D input refuse curve_history_output rather than record nothing.
    MatrixXd V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    MatrixXd V2(3, 3);
    V2 << 0, 0, 0, 1, 0, 0, 0, 1, 0;
    MatrixXi E2(3, 2);
    E2 << 0, 1, 1, 2, 2, 0;
    const MeshMap meshes{
        {"tets", io::geometry_data(V, T, CellType::Tetrahedron)},
        {"loop", io::geometry_data(V2, E2, CellType::Edge)}};
    for (const char* input : {"tets", "loop"}) {
        CAPTURE(input);
        CHECK_THROWS_WITH(
            simwild::simwild(
                {{"input", {input}},
                 {"curve_history_output", "recorded"},
                 {"max_iterations", 1},
                 {"num_threads", 1}},
                meshes),
            Catch::Matchers::ContainsSubstring("feature curves of a 3D surface input only"));
    }
}
