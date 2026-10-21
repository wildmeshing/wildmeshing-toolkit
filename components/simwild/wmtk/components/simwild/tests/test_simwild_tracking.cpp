// How to use simwild's tracking file: map your own points, or attributes, between the input
// surfaces and the tracked surface of the output tet mesh -- the interfaces between its tags. A
// point on the output is named by three vertices of the tet mesh and barycentric coordinates.

#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/components/surface_map/Tracking.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <igl/readOBJ.h>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <random>
#include <string>

namespace fs = std::filesystem;
using namespace wmtk;
using namespace wmtk::components;
using surface_map::InputPoint;
using surface_map::kNoId;
using surface_map::OutputPoint;
using surface_map::Tracking;

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
