// How to use shortest_edge_collapse's tracking file: map your own points between its input and its
// output. The output is the simplified surface itself, so a point on it is an output triangle and
// barycentric coordinates, as for isotropic_remeshing (see the tracking component's
// test_tracking.cpp).

#include <wmtk/components/shortest_edge_collapse/shortest_edge_collapse.hpp>
#include <wmtk/components/tracking/tracking.hpp>

#include <igl/readOBJ.h>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

namespace fs = std::filesystem;
using namespace wmtk::components;
using tracking::InputPoint;
using tracking::kNoId;
using tracking::OutputPoint;
using tracking::Tracking;

namespace {

Eigen::Vector3d
at(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const uint64_t f, const Eigen::Vector3d& b)
{
    return b[0] * V.row(F(f, 0)).transpose() + b[1] * V.row(F(f, 1)).transpose() +
           b[2] * V.row(F(f, 2)).transpose();
}

double mean_edge(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    double sum = 0;
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        for (int k = 0; k < 3; ++k) sum += (V.row(F(f, k)) - V.row(F(f, (k + 1) % 3))).norm();
    }
    return sum / double(3 * F.rows());
}

} // namespace

TEST_CASE("tracking-example-shortest-edge-collapse", "[shortest-edge-collapse][tracking]")
{
    const fs::path dir = fs::current_path() / "tracking_example_shortest-edge-collapse";
    fs::create_directories(dir);
    const fs::path input = fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj";
    const fs::path output = dir / "sphere_simplified.obj";
    const fs::path tracking_file = dir / "sphere.wmtkhist";

    // 1. Run shortest_edge_collapse with tracking on: history_output names the tracking file.
    shortest_edge_collapse::shortest_edge_collapse({
        {"application", "shortest_edge_collapse"},
        {"input", input.string()},
        {"output", output.string()},
        {"history_output", tracking_file.string()},
        {"target_rel", 0.3},
    });
    Eigen::MatrixXd V_in, V_out;
    Eigen::MatrixXi F_in, F_out;
    REQUIRE(igl::readOBJ(input.string(), V_in, F_in));
    REQUIRE(igl::readOBJ(output.string(), V_out, F_out));

    // 2. Load it. Its last state is the output file: the same triangles, in the same order.
    const Tracking tracking = Tracking::load(tracking_file.string());
    REQUIRE(!tracking.output_by_vertices());
    REQUIRE(tracking.input_faces().size() == size_t(F_in.rows()));
    const auto triangles = tracking.output_triangles();
    REQUIRE(triangles.size() == size_t(F_out.rows()));
    const auto& P = tracking.map().output_vertices();
    for (Eigen::Index f = 0; f < F_out.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            CHECK((P[triangles[f][k]] - V_out.row(F_out(f, k)).transpose()).norm() < 1e-6);
        }
    }

    // 3. Points on the input -- a triangle of the input file and barycentric coordinates -- to
    // the output and back. A point's image is where it went on the surface: near it.
    std::mt19937 rng(7);
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
    const double edge = mean_edge(V_out, F_out);
    double round_trip = 0, moved = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].face < uint64_t(F_out.rows()));
        REQUIRE(back[i].face == pts[i].face);
        round_trip = std::max(round_trip, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
        moved = std::max(
            moved,
            (at(V_out, F_out, there[i].face, there[i].bary) -
             at(V_in, F_in, pts[i].face, pts[i].bary))
                .norm());
    }
    CHECK(round_trip < 1e-8);
    CHECK(moved < 2 * edge);
    CHECK(tracking.map().map_warnings().total() == 0);

    // 4. The same from the command line, as the wmtk binary runs it: a file of points, one
    // `face b0 b1 b2` per line, mapped by the tracking application.
    const fs::path points = dir / "points.txt";
    {
        std::ofstream o(points);
        o.precision(17);
        for (const InputPoint& p : pts) {
            o << p.face << " " << p.bary[0] << " " << p.bary[1] << " " << p.bary[2] << "\n";
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
        int64_t face;
        Eigen::Vector3d b;
        ss >> face >> b[0] >> b[1] >> b[2];
        REQUIRE(i < there.size());
        same += uint64_t(face) == there[i].face && (b - there[i].bary).norm() < 1e-12;
    }
    CHECK(i == pts.size());
    CHECK(same == pts.size());
}
