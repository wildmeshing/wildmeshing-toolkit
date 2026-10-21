#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/surface_map/SurfaceMap.hpp>
#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/history/SurfaceHistory.hpp>

#include <algorithm>
#include <filesystem>
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
    // The input as inserted, through the optimization, to the output's tracked surface: the
    // history ends exactly there, and the map between the two is a bijection.
    const fs::path input = fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj";
    REQUIRE(fs::exists(input));
    const fs::path dir = fs::current_path() / "tetwild_history_test";
    fs::create_directories(dir);
    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        const fs::path out = dir / fmt::format("sphere_{}", threads);
        const fs::path hist = dir / fmt::format("sphere_{}.wmtkhist", threads);
        const auto exported = tetwild::tetwild_with_export({
            {"application", "tetwild"},
            {"input", {input.string()}},
            {"output", out.string()},
            {"history_output", hist.string()},
            {"max_iterations", 5},
            {"num_threads", threads},
            {"write_vtu", false},
        });
        const auto h = history::SurfaceHistory::load(hist.string());
        const surface_map::SurfaceMap map(h);

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
        const auto back = map.to_output(map.to_input(pts));
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
        CHECK(err < 1e-8);
    }
}
