#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/surface_map/SurfaceMap.hpp>
#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/history/SurfaceHistory.hpp>

#include <igl/readOBJ.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>

namespace fs = std::filesystem;
using namespace wmtk;
using namespace wmtk::components;

namespace {

/// A face by its corner positions, sorted: the same in the history and in the exported mesh.
using Key = std::array<std::array<double, 3>, 3>;

Key key_of(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
    Key k = {{{{a[0], a[1], a[2]}}, {{b[0], b[1], b[2]}}, {{c[0], c[1], c[2]}}}};
    std::sort(k.begin(), k.end());
    return k;
}

} // namespace

TEST_CASE("tetwild-surface-history-replays-to-the-tracked-surface", "[tetwild][history]")
{
    // Two spheres, from two files: the history runs from the files as read -- the second a
    // triangle soup, every vertex repeated per face, so the weld has work to do -- through the
    // insertion and the optimization to the output's tracked surface. It ends exactly there,
    // the map is a bijection, and an output face maps onto the file of the sphere it is on.
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
            {"write_vtu", false},
        });
        const auto h = history::SurfaceHistory::load(hist.string());
        const surface_map::SurfaceMap map(h);
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

        std::vector<surface_map::SurfacePoint> pts;
        for (uint64_t f = 0; f < map.output_faces().size(); ++f) {
            if (map.output_face_alive()[f]) pts.push_back({f, Eigen::Vector3d::Constant(1. / 3)});
        }
        const auto there = map.to_input(pts);
        size_t wrong_file = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            const auto& t = map.output_faces()[pts[i].face];
            const Eigen::Vector3d c = (V[t[0]] + V[t[1]] + V[t[2]]) / 3;
            const bool second = (c - centre1).norm() < (c - centre0).norm();
            REQUIRE(there[i].face < map.input_faces().size());
            wrong_file += (there[i].face >= uint64_t(SF.rows())) != second;
        }
        CHECK(wrong_file == 0);
        const auto back = map.to_output(there);
        double err = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            REQUIRE(back[i].face == pts[i].face);
            err = std::max(err, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
        }
        const auto& w = map.build_warnings();
        INFO(
            "build warnings " << w.total() << " (not a disk " << w.not_a_disk << ", flat "
                              << w.flat_triangle << ", inverted " << w.inverted << ", merged "
                              << w.merged_faces << ", replaced " << w.replaced << "), round trip "
                              << err);
        CHECK(w.total() == 0);
        CHECK(map.map_warnings().total() == 0);
        CHECK(err < 1e-8);
    }
}

TEST_CASE("tetwild-surface-history-octocat", "[tetwild][history][.]")
{
    // A real input, welded and simplified -- with the link condition, so the simplification
    // keeps the topology the map needs -- inserted and optimized with preserve_topology.
    const fs::path input = fs::path(WMTK_DATA_DIR) / "models" / "Octocat.obj";
    REQUIRE(fs::exists(input));
    const fs::path dir = fs::current_path() / "tetwild_history_test";
    fs::create_directories(dir);
    const fs::path hist = dir / "octocat.wmtkhist";
    tetwild::tetwild_with_export({
        {"application", "tetwild"},
        {"input", {input.string()}},
        {"output", (dir / "octocat").string()},
        {"history_output", hist.string()},
        {"num_threads", 8},
        {"preserve_topology", true},
        {"simplify_use_link_condition", true},
        {"write_vtu", false},
    });
    const surface_map::SurfaceMap map(history::SurfaceHistory::load(hist.string()));
    std::vector<surface_map::SurfacePoint> pts;
    for (uint64_t f = 0; f < map.output_faces().size(); ++f) {
        if (map.output_face_alive()[f]) pts.push_back({f, Eigen::Vector3d::Constant(1. / 3)});
    }
    const auto back = map.to_output(map.to_input(pts));
    double err = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(back[i].face == pts[i].face);
        err = std::max(err, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
    }
    CHECK(map.build_warnings().total() == 0);
    CHECK(map.map_warnings().total() == 0);
    CHECK(err < 1e-8);
}
