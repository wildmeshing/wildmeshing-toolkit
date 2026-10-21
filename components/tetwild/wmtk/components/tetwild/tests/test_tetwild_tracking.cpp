// How to use tetwild's tracking file: map your own points, or attributes, between the input
// surface and the tracked surface of the output tet mesh. A point on the output is named by
// three vertices of the tet mesh -- the corners of the tracked face holding it -- and
// barycentric coordinates, since the tracked surface is no element of the output file.

#include <wmtk/components/surface_map/Tracking.hpp>
#include <wmtk/components/surface_map/surface_map.hpp>
#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/utils/io.hpp>

#include <igl/readOBJ.h>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>

namespace fs = std::filesystem;
using namespace wmtk;
using namespace wmtk::components;
using surface_map::InputPoint;
using surface_map::kNoId;
using surface_map::OutputPoint;
using surface_map::Tracking;

TEST_CASE("tracking-example-tetwild", "[tetwild][tracking]")
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
        MshData msh;
        msh.load(output.string() + "_final.msh");
        tet_vertices.resize(msh.get_num_tet_vertices());
        msh.extract_tet_vertices([&](size_t i, double x, double y, double z) {
            tet_vertices[i] = Eigen::Vector3d(x, y, z);
        });
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
    surface_map::surface_map({
        {"application", "surface_map"},
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
