// How to use tetwild's tracking file: map your own points, or attributes, between the input
// surface and the tracked surface of the output tet mesh. A point on the output is named by
// three vertices of the tet mesh -- the corners of the tracked face holding it -- and
// barycentric coordinates, since the tracked surface is no element of the output file.

#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/history/VolumeHistory.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <igl/readOBJ.h>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
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

TEST_CASE("tracking-example-tetwild", "[tetwild][tracking]" + release_only)
{
    const fs::path dir = fs::current_path() / "tracking_example_tetwild";
    fs::create_directories(dir);
    const fs::path input = fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj";
    const fs::path output = dir / "sphere";
    const fs::path tracking_file = dir / "sphere.wmtkhist";

    // 1. Run tetwild with tracking on: history_output names the tracking file. The tet mesh is
    // written to <output>_final.msh, whose vertices the tracking names. preserve_topology keeps
    // the map a bijection.
    tetwild::tetwild({
        {"application", "tetwild"},
        {"input", {input.string()}},
        {"output", output.string()},
        {"history_output", tracking_file.string()},
        {"preserve_topology", true},
        {"max_iterations", 3},
        {"write_vtu", false},
    });
    std::vector<Eigen::Vector3d> tet_vertices;
    {
        // Read as wm.read_mesh reads it: every vertex, in the file's row order.
        const MeshData msh = io::read_mesh(output.string() + "_final.msh");
        for (Eigen::Index i = 0; i < msh.V.rows(); ++i) {
            tet_vertices.push_back(msh.V.row(i).transpose());
        }
    }
    Eigen::MatrixXd V_in;
    Eigen::MatrixXi F_in;
    REQUIRE(igl::readOBJ(input.string(), V_in, F_in));
    const auto on_input = [&](const InputPoint& p) {
        return Eigen::Vector3d(
            p.bary[0] * V_in.row(F_in(p.face, 0)) + p.bary[1] * V_in.row(F_in(p.face, 1)) +
            p.bary[2] * V_in.row(F_in(p.face, 2)));
    };
    const auto on_output = [&](const OutputPoint& p) {
        return Eigen::Vector3d(
            p.bary[0] * tet_vertices[p.vertices[0]] + p.bary[1] * tet_vertices[p.vertices[1]] +
            p.bary[2] * tet_vertices[p.vertices[2]]);
    };

    // 2. Load it, and map points on the input -- a triangle of the input file and barycentric
    // coordinates -- to the output, and back.
    const Tracking tracking = Tracking::load(tracking_file.string());
    REQUIRE(tracking.output_by_vertices()); // a tet output: points are vertex triplets
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(0, 1);
    std::vector<InputPoint> pts;
    for (int i = 0; i < 500; ++i) {
        double a = u(rng), b = u(rng);
        if (a + b > 1) {
            a = 1 - a;
            b = 1 - b;
        }
        pts.push_back({uint64_t(rng() % F_in.rows()), Eigen::Vector3d(1 - a - b, a, b)});
    }
    const std::vector<OutputPoint> there = tracking.to_output(pts);
    const std::vector<InputPoint> back = tracking.to_input(there);
    double mean_edge = 0;
    for (Eigen::Index f = 0; f < F_in.rows(); ++f) {
        mean_edge += (V_in.row(F_in(f, 0)) - V_in.row(F_in(f, 1))).norm();
    }
    mean_edge /= F_in.rows();
    double round_trip = 0, moved = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].vertices[0] != kNoId);
        for (const uint64_t v : there[i].vertices) REQUIRE(v < tet_vertices.size());
        REQUIRE(back[i].face == pts[i].face);
        round_trip = std::max(round_trip, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
        moved = std::max(moved, (on_output(there[i]) - on_input(pts[i])).norm());
    }
    CHECK(round_trip < 1e-8);
    CHECK(moved < 2 * mean_edge);

    // The triplets are tracked faces of the tet mesh, at the positions the tracking has.
    for (const auto& t : tracking.output_triangles()) {
        for (const uint64_t v : t) REQUIRE(v < tet_vertices.size());
    }

    // 3. Remap an attribute of your own, per vertex on the input, onto the vertices of the tet
    // mesh's tracked surface: each is a corner of a tracked face; map it to the input there and
    // interpolate.
    const auto f = [](const Eigen::Vector3d& p) { return p.x() + 2 * p.y() + 3 * p.z(); };
    std::map<uint64_t, OutputPoint> corner_of;
    for (const auto& t : tracking.output_triangles()) {
        for (int k = 0; k < 3; ++k) corner_of[t[k]] = {kNoId, t, Eigen::Vector3d::Unit(k)};
    }
    std::vector<OutputPoint> corners;
    for (const auto& [v, p] : corner_of) corners.push_back(p);
    const std::vector<InputPoint> sources = tracking.to_input(corners);
    double worst = 0;
    size_t k = 0;
    for (const auto& [v, p] : corner_of) {
        const InputPoint& s = sources[k++];
        REQUIRE(s.face != kNoId);
        const double value = s.bary[0] * f(V_in.row(F_in(s.face, 0)).transpose()) +
                             s.bary[1] * f(V_in.row(F_in(s.face, 1)).transpose()) +
                             s.bary[2] * f(V_in.row(F_in(s.face, 2)).transpose());
        worst = std::max(worst, std::abs(value - f(tet_vertices[v])));
    }
    CHECK(worst < 4 * 2 * mean_edge); // |grad f| < 4

    // 4. The same from the command line, as the wmtk binary runs it: points on the output, one
    // `v0 v1 v2 b0 b1 b2` per line (the corners in any order), mapped back to the input.
    const fs::path points = dir / "points_on_output.txt";
    {
        std::ofstream o(points);
        o.precision(17);
        for (const OutputPoint& p : there) {
            // The corners listed backwards, to show the order is free.
            o << p.vertices[2] << " " << p.vertices[1] << " " << p.vertices[0] << " " << p.bary[2]
              << " " << p.bary[1] << " " << p.bary[0] << "\n";
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
        int64_t face;
        Eigen::Vector3d b;
        ss >> face >> b[0] >> b[1] >> b[2];
        REQUIRE(i < pts.size());
        same += uint64_t(face) == pts[i].face && (b - pts[i].bary).norm() < 1e-8;
    }
    CHECK(i == pts.size());
    CHECK(same == pts.size());
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

/// An icosphere of radius `r`: the icosahedron's faces split in four `level` times, outward.
MeshData icosphere(const int level, const double r)
{
    const double t = (1 + std::sqrt(5.)) / 2;
    std::vector<Eigen::Vector3d> V = {
        {-1, t, 0},
        {1, t, 0},
        {-1, -t, 0},
        {1, -t, 0},
        {0, -1, t},
        {0, 1, t},
        {0, -1, -t},
        {0, 1, -t},
        {t, 0, -1},
        {t, 0, 1},
        {-t, 0, -1},
        {-t, 0, 1}};
    std::vector<std::array<int64_t, 3>> F = {
        {{0, 11, 5}}, {{0, 5, 1}},  {{0, 1, 7}},   {{0, 7, 10}}, {{0, 10, 11}},
        {{1, 5, 9}},  {{5, 11, 4}}, {{11, 10, 2}}, {{10, 7, 6}}, {{7, 1, 8}},
        {{3, 9, 4}},  {{3, 4, 2}},  {{3, 2, 6}},   {{3, 6, 8}},  {{3, 8, 9}},
        {{4, 9, 5}},  {{2, 4, 11}}, {{6, 2, 10}},  {{8, 6, 7}},  {{9, 8, 1}}};
    for (int l = 0; l < level; ++l) {
        std::map<std::pair<int64_t, int64_t>, int64_t> mid;
        const auto midpoint = [&](int64_t a, int64_t b) {
            const auto key = std::minmax(a, b);
            const auto [it, fresh] = mid.emplace(key, int64_t(V.size()));
            if (fresh) V.push_back((V[a] + V[b]) / 2);
            return it->second;
        };
        std::vector<std::array<int64_t, 3>> next;
        for (const auto& f : F) {
            const int64_t a = midpoint(f[0], f[1]), b = midpoint(f[1], f[2]),
                          c = midpoint(f[2], f[0]);
            next.push_back({{f[0], a, c}});
            next.push_back({{f[1], b, a}});
            next.push_back({{f[2], c, b}});
            next.push_back({{a, b, c}});
        }
        F = std::move(next);
    }
    MeshData m;
    m.V.resize(Eigen::Index(V.size()), 3);
    for (size_t i = 0; i < V.size(); ++i) m.V.row(Eigen::Index(i)) = r * V[i].normalized();
    auto& T = m.cells[CellType::Triangle].connectivity;
    T.resize(Eigen::Index(F.size()), 3);
    for (size_t i = 0; i < F.size(); ++i) {
        for (int k = 0; k < 3; ++k) T(Eigen::Index(i), k) = F[i][k];
    }
    return m;
}

/// The small tetwild run whose volume history the tests map: an icosphere of 162 vertices,
/// three iterations, about a second.
nlohmann::json small_volume_run()
{
    return {
        {"max_iterations", 3},
        {"num_threads", 1},
        {"write_vtu", false},
        {"history_output", "recorded"},
        {"volume_history_output", "recorded_volume"}};
}

} // namespace

TEST_CASE("tracking-example-tetwild-volume", "[tetwild][tracking]" + release_only)
{
    const fs::path dir = fs::current_path() / "tracking_example_tetwild_volume";
    fs::create_directories(dir);
    const fs::path input = fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj";
    const fs::path output = dir / "sphere";

    // 1. Run tetwild with the surface history and the volume history: the whole volume is
    // tracked, from <output>_history_input.msh, where the optimization starts, to
    // <output>_history_output.msh, the mesh before the filter; <output>_final.msh is the part
    // the filter keeps.
    tetwild::tetwild({
        {"application", "tetwild"},
        {"input", {input.string()}},
        {"output", output.string()},
        {"history_output", output.string() + ".wmtkhist"},
        {"volume_history_output", output.string() + ".wmtkvol"},
        {"filter", "input"},
        {"preserve_topology", true},
        {"max_iterations", 3},
        {"write_vtu", false},
    });
    const auto load = [&](const std::string& end) {
        return tracking::VolumeTracking::load(
            output.string() + ".wmtkhist",
            output.string() + ".wmtkvol",
            output.string() + "_history_input.msh",
            output.string() + end);
    };
    const tracking::VolumeTracking tracking = load("_history_output.msh");
    const auto& in = tracking.input_mesh();
    const auto& out = tracking.output_mesh();
    const auto position = [](const tracking::TetMeshData& m, const tracking::TetPoint& p) {
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) x += p.bary[k] * m.vertices[m.tets[p.tet][k]];
        return x;
    };

    // 2. Points near the sphere (radius 0.7), inside and out: every one lands in the mesh before
    // the filter and comes back; the sphere's surface stays the sphere's, so a point inside
    // stays inside.
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(0, 1);
    std::vector<tracking::TetPoint> pts;
    while (pts.size() < 2000) {
        tracking::TetPoint p{rng() % in.tets.size()};
        const double r = position(in, p).norm();
        if (r < 0.5 || r > 0.85) continue;
        p.bary = Eigen::Vector4d(u(rng), u(rng), u(rng), u(rng));
        p.bary /= p.bary.sum();
        pts.push_back(p);
    }
    const auto there = tracking.to_output(pts);
    const auto back = tracking.to_input(there);
    CHECK(tracking.map().stats().failed == 0);
    CHECK(tracking.map().stats().surface_not_kept == 0);
    size_t moved = 0;
    double round_trip = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].tet != tracking::kNoId);
        REQUIRE(back[i].tet != tracking::kNoId);
        moved += (position(out, there[i]) - position(in, pts[i])).norm() > 0;
        round_trip = std::max(round_trip, (position(in, back[i]) - position(in, pts[i])).norm());
    }
    CHECK(moved > pts.size() / 10);
    CHECK(round_trip < 1e-6); // rounding, as in tracking-example-simwild-volume
    // A round trip the identity passes too: the points that move keep their orientation.
    const auto [moving, kept_orientation] = orientation_kept(tracking, pts);
    CHECK(moving > 0);
    CHECK(kept_orientation == moving);

    // 3. Against the filtered output: the points well inside the sphere have a tet there, those
    // well outside none (in between, the sphere's facets decide).
    const tracking::VolumeTracking kept = load("_final.msh");
    const auto in_final = kept.to_output(pts);
    size_t inside = 0, inside_found = 0, outside = 0, outside_found = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const double r = position(in, pts[i]).norm();
        const bool found = in_final[i].tet != tracking::kNoId;
        if (r < 0.6) {
            ++inside;
            inside_found += found;
        } else if (r > 0.75) {
            ++outside;
            outside_found += found;
        }
    }
    CHECK(inside > 0);
    CHECK(inside_found == inside);
    CHECK(outside > 0);
    CHECK(outside_found == 0);
}

TEST_CASE("tetwild-volume-history-refusals", "[tetwild][tracking]")
{
    // What the volume history cannot follow is refused before any work: the coarsening pass's
    // local smoothing, which moves a ball of vertices inside one collapse, and a volume history
    // without the surface history it follows.
    const fs::path dir = fs::current_path() / "tetwild_volume_history_refusals";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const fs::path output = dir / "sphere";
    const nlohmann::json base = {
        {"application", "tetwild"},
        {"input", {(fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj").string()}},
        {"output", output.string()},
        {"history_output", output.string() + ".wmtkhist"},
        {"volume_history_output", output.string() + ".wmtkvol"},
        {"max_iterations", 1},
        {"write_vtu", false},
    };
    nlohmann::json j = base;
    j["coarsen_pass"] = true;
    j["coarsen_local_smoothing_passes"] = 2;
    CHECK_THROWS_AS(tetwild::tetwild(j), std::runtime_error);
    j = base;
    j["history_output"] = "";
    CHECK_THROWS_AS(tetwild::tetwild(j), std::runtime_error);
    j = base;
    j["use_legacy_code"] = true;
    CHECK_THROWS_AS(tetwild::tetwild(j), std::runtime_error);
    CHECK_FALSE(fs::exists(output.string() + ".wmtkvol"));
}

TEST_CASE("tetwild-volume-history-memory-route", "[tetwild][tracking]" + release_only)
{
    // On the memory route the volume history comes back in the report, as the bytes of its file,
    // next to the surface history's, and the meshes it starts and ends at as the output meshes
    // history_input and history_output: what the file route writes, and no file. From them alone
    // a VolumeTracking maps as the one the files load.
    const fs::path dir = fs::current_path() / "tetwild_volume_history_memory_route";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const MeshData sphere = icosphere(2, 0.7);
    {
        std::ofstream obj(dir / "sphere.obj");
        obj.precision(17);
        for (Eigen::Index v = 0; v < sphere.V.rows(); ++v) {
            obj << "v " << sphere.V(v, 0) << " " << sphere.V(v, 1) << " " << sphere.V(v, 2) << "\n";
        }
        const auto& F = sphere.cell(CellType::Triangle).connectivity;
        for (Eigen::Index f = 0; f < F.rows(); ++f) {
            obj << "f " << F(f, 0) + 1 << " " << F(f, 1) + 1 << " " << F(f, 2) + 1 << "\n";
        }
    }
    nlohmann::json options = small_volume_run();
    options["input"] = "sphere";
    const auto [outputs, report] = tetwild::tetwild(options, {{"sphere", sphere}});
    CHECK(!fs::exists("recorded"));
    CHECK(!fs::exists("recorded_volume"));
    REQUIRE(report.at("history").is_binary());
    REQUIRE(report.at("volume_history").is_binary());
    REQUIRE(outputs.count("history_input"));
    REQUIRE(outputs.count("history_output"));
    const auto& surface_bytes = report.at("history").get_binary();
    const auto& volume_bytes = report.at("volume_history").get_binary();
    const auto volume = history::VolumeHistory::from_bytes(volume_bytes);
    CHECK(!volume.edits.empty());
    CHECK(volume.regions.size() == 1);
    CHECK(
        volume.surface == history::surface_key(history::SurfaceHistory::from_bytes(surface_bytes)));

    // The file route: the same volume history, byte for byte, and the same meshes.
    const fs::path output = dir / "sphere";
    nlohmann::json file = small_volume_run();
    file["application"] = "tetwild";
    file["input"] = {(dir / "sphere.obj").string()};
    file["output"] = output.string();
    file["history_output"] = output.string() + ".wmtkhist";
    file["volume_history_output"] = output.string() + ".wmtkvol";
    tetwild::tetwild(file);
    CHECK(history::read_history(output.string() + ".wmtkvol") == volume_bytes);
    for (const auto& [key, suffix] :
         {std::pair<std::string, std::string>{"history_input", "_history_input.msh"},
          {"history_output", "_history_output.msh"}}) {
        const MeshData written = io::read_mesh(output.string() + suffix);
        const MeshData& returned = outputs.at(key);
        CHECK(returned.V == written.V);
        CHECK(
            returned.cell(CellType::Tetrahedron).connectivity ==
            written.cell(CellType::Tetrahedron).connectivity);
    }

    const tracking::VolumeTracking memory(
        history::SurfaceHistory::from_bytes(surface_bytes),
        history::VolumeHistory::from_bytes(volume_bytes),
        outputs.at("history_input"),
        outputs.at("history_output"));
    const tracking::VolumeTracking files = tracking::VolumeTracking::load(
        output.string() + ".wmtkhist",
        output.string() + ".wmtkvol",
        output.string() + "_history_input.msh",
        output.string() + "_history_output.msh");
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> u(0.05, 1);
    std::vector<tracking::TetPoint> pts;
    for (int i = 0; i < 500; ++i) {
        tracking::TetPoint p{rng() % memory.input_mesh().tets.size()};
        p.bary = Eigen::Vector4d(u(rng), u(rng), u(rng), u(rng));
        p.bary /= p.bary.sum();
        pts.push_back(p);
    }
    const auto there = memory.to_output(pts);
    const auto from_files = files.to_output(pts);
    const auto back = memory.to_input(there);
    const auto& in = memory.input_mesh();
    const auto position = [&in](const tracking::TetPoint& p) {
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) x += p.bary[k] * in.vertices[in.tets[p.tet][k]];
        return x;
    };
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].tet != tracking::kNoId);
        CHECK(there[i].tet == from_files[i].tet);
        CHECK(there[i].bary == from_files[i].bary);
        REQUIRE(back[i].tet != tracking::kNoId);
        // no tet of an operation is inverted at the positions recorded: back to a rounding
        CHECK((position(back[i]) - position(pts[i])).norm() < 1e-12);
    }
    CHECK(memory.map().stats().inverted_ops == 0);

    // Without volume_history_output, neither the volume history nor its meshes.
    options.erase("volume_history_output");
    const auto [plain, plain_report] = tetwild::tetwild(options, {{"sphere", sphere}});
    CHECK(!plain_report.contains("volume_history"));
    CHECK(!plain.count("history_input"));
    CHECK(!plain.count("history_output"));
}

TEST_CASE("tetwild-volume-coarse-round-trip", "[tetwild][tracking]" + release_only)
{
    // A known limitation, pinned (docs/tracking.md, "Volumes"). On an input this coarse -- the
    // icosahedron split once, 42 vertices -- the insertion leaves tets that the positions the
    // volume history holds, the exact ones rounded to doubles, turn flat or inverted
    // (VolumeMapStats::inverted_ops): the tets there overlap, a position is in two, and a point
    // mapped there and back can come back off, by up to the size of the tets around. Every
    // operation maps, and the other points come back to a rounding.
    nlohmann::json options = small_volume_run();
    options["input"] = "sphere";
    options["length_rel"] = 0.3;
    options["eps_rel"] = 0.02;
    options["max_iterations"] = 1;
    options["preserve_topology"] = true;
    const auto [outputs, report] = tetwild::tetwild(options, {{"sphere", icosphere(1, 1)}});
    const tracking::VolumeTracking t(
        history::SurfaceHistory::from_bytes(report.at("history").get_binary()),
        history::VolumeHistory::from_bytes(report.at("volume_history").get_binary()));
    const MeshData& start = outputs.at("history_input");
    const auto& T = start.cell(CellType::Tetrahedron).connectivity;
    const double diagonal = (start.V.colwise().maxCoeff() - start.V.colwise().minCoeff()).norm();
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> u(0.05, 1);
    std::vector<Eigen::Vector3d> pts;
    for (int i = 0; i < 3000; ++i) {
        const Eigen::Index r = Eigen::Index(rng() % uint32_t(T.rows()));
        Eigen::Vector4d b(u(rng), u(rng), u(rng), u(rng));
        b /= b.sum();
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) x += b[k] * start.V.row(T(r, k)).transpose();
        pts.push_back(x);
    }
    const auto there = t.map_positions(pts, true);
    std::vector<Eigen::Vector3d> xyz;
    std::vector<uint32_t> regions;
    for (const auto& p : there) {
        xyz.push_back(p.position);
        regions.push_back(p.region);
    }
    const auto back = t.map_positions(xyz, false, regions);
    size_t off = 0;
    double worst = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const double d = (back[i].position - pts[i]).norm() / diagonal;
        off += d > 1e-9;
        worst = std::max(worst, d);
    }
    const auto stats = t.map().stats();
    CHECK(stats.failed == 0);
    CHECK(stats.inverted_ops > 0);
    // 17 of 3000 off, the worst by 1.3% of the diagonal, and 61 such operations, when this was
    // written; the bounds leave room for another platform's run
    CHECK(off < pts.size() / 50);
    CHECK(worst < 0.05);
}
