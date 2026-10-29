// How to use tetwild's tracking file: map your own points, or attributes, between the input
// surface and the tracked surface of the output tet mesh. A point on the output is named by
// three vertices of the tet mesh -- the corners of the tracked face holding it -- and
// barycentric coordinates, since the tracked surface is no element of the output file.

#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <igl/readOBJ.h>
#include <catch2/catch_test_macros.hpp>

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
