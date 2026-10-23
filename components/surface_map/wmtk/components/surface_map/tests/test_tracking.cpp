// How to use an application's tracking file: map your own points, or attributes, between its
// input and its output. This one uses isotropic_remeshing, whose output is a surface: a point on
// it is an output triangle and barycentric coordinates. See the tetwild and simwild tests for
// tet outputs, where it is three vertices of the tet mesh instead.

#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/components/surface_map/Tracking.hpp>
#include <wmtk/components/surface_map/surface_map.hpp>

#include <igl/readOBJ.h>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>

namespace fs = std::filesystem;
using namespace wmtk::components;
using surface_map::InputPoint;
using surface_map::kNoId;
using surface_map::OutputPoint;
using surface_map::Tracking;

namespace {

Eigen::Vector3d
at(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const uint64_t f, const Eigen::Vector3d& b)
{
    return b[0] * V.row(F(f, 0)).transpose() + b[1] * V.row(F(f, 1)).transpose() +
           b[2] * V.row(F(f, 2)).transpose();
}

std::vector<InputPoint> random_points(const size_t n_faces, const size_t n, const unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<uint64_t> face(0, n_faces - 1);
    std::uniform_real_distribution<double> u(0, 1);
    std::vector<InputPoint> pts;
    for (size_t i = 0; i < n; ++i) {
        double a = u(rng), b = u(rng);
        if (a + b > 1) {
            a = 1 - a;
            b = 1 - b;
        }
        pts.push_back({face(rng), Eigen::Vector3d(1 - a - b, a, b)});
    }
    return pts;
}

} // namespace

TEST_CASE("tracking-example-isotropic", "[surface_map][tracking]")
{
    const fs::path dir = fs::current_path() / "tracking_example_isotropic";
    fs::create_directories(dir);
    const fs::path input = fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj";
    const fs::path output = dir / "sphere_remeshed.obj";
    const fs::path tracking_file = dir / "sphere.wmtkhist";

    // 1. Run the application with tracking on: history_output names the tracking file.
    isotropic_remeshing::isotropic_remeshing({
        {"application", "isotropic_remeshing"},
        {"input", input.string()},
        {"output", output.string()},
        {"history_output", tracking_file.string()},
        {"length_rel", 0.08},
        {"max_iterations", 3},
    });
    Eigen::MatrixXd V_in, V_out;
    Eigen::MatrixXi F_in, F_out;
    REQUIRE(igl::readOBJ(input.string(), V_in, F_in));
    REQUIRE(igl::readOBJ(output.string(), V_out, F_out));

    // 2. Load it, and map points on the input -- a triangle of the input file and barycentric
    // coordinates -- to the output, and back.
    const Tracking tracking = Tracking::load(tracking_file.string());
    REQUIRE(!tracking.output_by_vertices()); // a surface output names its own triangles
    REQUIRE(tracking.input_faces().size() == size_t(F_in.rows()));
    const std::vector<InputPoint> pts = random_points(F_in.rows(), 500, 7);
    const std::vector<OutputPoint> there = tracking.to_output(pts);
    const std::vector<InputPoint> back = tracking.to_input(there);

    double mean_edge = 0;
    for (Eigen::Index f = 0; f < F_in.rows(); ++f) {
        mean_edge += (V_in.row(F_in(f, 0)) - V_in.row(F_in(f, 1))).norm();
    }
    mean_edge /= F_in.rows();
    double round_trip = 0, moved = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(there[i].face != kNoId);
        REQUIRE(there[i].face < uint64_t(F_out.rows()));
        REQUIRE(back[i].face == pts[i].face);
        round_trip = std::max(round_trip, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
        // The image is where the point went on the surface: near it, the remeshing having moved
        // the surface's vertices along it by about an edge at most.
        moved = std::max(
            moved,
            (at(V_out, F_out, there[i].face, there[i].bary) -
             at(V_in, F_in, pts[i].face, pts[i].bary))
                .norm());
    }
    CHECK(round_trip < 1e-8);
    CHECK(moved < 2 * mean_edge);
    CHECK(tracking.map().map_warnings().total() == 0);

    // 3. Remap an attribute of your own: per vertex on the input, wanted per vertex on the
    // output. Each output vertex is a point of an output triangle at one corner; map it to the
    // input and interpolate there.
    const auto f = [](const Eigen::Vector3d& p) { return p.x() + 2 * p.y() + 3 * p.z(); };
    std::vector<double> attribute(V_in.rows());
    for (Eigen::Index v = 0; v < V_in.rows(); ++v) attribute[v] = f(V_in.row(v).transpose());
    std::vector<OutputPoint> corners(V_out.rows());
    for (Eigen::Index t = 0; t < F_out.rows(); ++t) {
        for (int k = 0; k < 3; ++k) {
            corners[F_out(t, k)].face = uint64_t(t);
            corners[F_out(t, k)].bary = Eigen::Vector3d::Unit(k);
        }
    }
    const std::vector<InputPoint> sources = tracking.to_input(corners);
    double worst = 0;
    for (Eigen::Index v = 0; v < V_out.rows(); ++v) {
        const InputPoint& s = sources[v];
        REQUIRE(s.face != kNoId);
        double value = 0;
        for (int k = 0; k < 3; ++k) value += s.bary[k] * attribute[F_in(s.face, k)];
        // A linear attribute comes back as itself, up to how far the vertex moved on the surface.
        worst = std::max(worst, std::abs(value - f(V_out.row(v).transpose())));
    }
    CHECK(worst < 4 * 2 * mean_edge); // |grad f| < 4

    // 4. The same from the command line, as the wmtk binary runs it: a file of points, one
    // `face b0 b1 b2` per line, mapped by the surface_map application.
    const fs::path points = dir / "points.txt";
    {
        std::ofstream o(points);
        o.precision(17);
        o << "# input triangle, then barycentric coordinates\n";
        for (const InputPoint& p : pts) {
            o << p.face << " " << p.bary[0] << " " << p.bary[1] << " " << p.bary[2] << "\n";
        }
    }
    const fs::path mapped = dir / "points_on_output.txt";
    surface_map::surface_map({
        {"application", "surface_map"},
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
