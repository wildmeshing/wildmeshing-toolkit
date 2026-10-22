#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/components/tracking/SurfaceMap.hpp>
#include <wmtk/components/tracking/surface_io.hpp>
#include <wmtk/components/tracking/transfer.hpp>
#include <wmtk/history/SurfaceHistory.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <igl/readOBJ.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>

namespace fs = std::filesystem;
using namespace wmtk;
using namespace wmtk::components;

namespace {

/// Whole runs of the application, too slow for the Debug CI jobs: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

/// A face by its corner positions, sorted: the same in the history and in the exported mesh.
using Key = std::array<std::array<double, 3>, 3>;

Key key_of(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
    Key k = {{{{a[0], a[1], a[2]}}, {{b[0], b[1], b[2]}}, {{c[0], c[1], c[2]}}}};
    std::sort(k.begin(), k.end());
    return k;
}

} // namespace

TEST_CASE(
    "tetwild-surface-history-replays-to-the-tracked-surface",
    "[tetwild][history]" + release_only)
{
    // Two spheres, from two files: the history runs from the files as read -- the second a
    // triangle soup, every vertex repeated per face, so the weld has work to do -- through the
    // insertion and the optimization to the output's tracked surface. It ends exactly there,
    // the map is a bijection, and an output face maps onto the file of the sphere it is on.
    // Under preserve_topology: without it, a run may weld or merge a few faces, and the map is
    // then rightly not a bijection there. A serial run is the same on every platform and is
    // held to that exactly; a parallel one, whose operations the schedule orders, may lose or
    // misplace one face in a hundred.
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
            {"preserve_topology", true},
            {"write_vtu", false},
        });
        const auto h = history::SurfaceHistory::load(hist.string());
        const tracking::SurfaceMap map(h);
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

        std::vector<tracking::SurfacePoint> pts;
        for (uint64_t f = 0; f < map.output_faces().size(); ++f) {
            if (map.output_face_alive()[f]) pts.push_back({f, Eigen::Vector3d::Constant(1. / 3)});
        }
        tracking::MapWarnings back_warnings;
        const auto there = map.to_input(pts);
        const auto back = map.to_output(there, &back_warnings);
        size_t lost = 0, wrong_file = 0;
        double err = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            const auto& t = map.output_faces()[pts[i].face];
            const Eigen::Vector3d c = (V[t[0]] + V[t[1]] + V[t[2]]) / 3;
            const bool second = (c - centre1).norm() < (c - centre0).norm();
            if (there[i].face >= map.input_faces().size() || back[i].face != pts[i].face) {
                ++lost;
                continue;
            }
            wrong_file += (there[i].face >= uint64_t(SF.rows())) != second;
            err = std::max(err, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
        }
        const auto& w = map.build_warnings();
        INFO(
            "build warnings " << w.total() << " (not a disk " << w.not_a_disk << ", flat "
                              << w.flat_triangle << ", inverted " << w.inverted << ", merged "
                              << w.merged_faces << ", replaced " << w.replaced << "), " << lost
                              << " lost, " << wrong_file << " on the wrong file, of " << pts.size()
                              << "; round trip " << err);
        if (threads == 0) {
            CHECK(lost == 0);
            CHECK(wrong_file == 0);
            CHECK(w.total() == 0);
            CHECK(back_warnings.total() == 0);
            CHECK(err < 1e-8);
        } else {
            CHECK(100 * (lost + wrong_file) <= pts.size());
        }
    }
}

TEST_CASE("tetwild-surface-history-octocat", "[tetwild][history][.]")
{
    // A real input, welded, simplified, inserted and optimized with preserve_topology, which
    // holds the simplification to the link condition too: the topology the map needs.
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
        {"write_vtu", false},
    });
    const tracking::SurfaceMap map(history::SurfaceHistory::load(hist.string()));
    std::vector<tracking::SurfacePoint> pts;
    for (uint64_t f = 0; f < map.output_faces().size(); ++f) {
        if (map.output_face_alive()[f]) pts.push_back({f, Eigen::Vector3d::Constant(1. / 3)});
    }
    tracking::MapWarnings back_warnings;
    const auto back = map.to_output(map.to_input(pts), &back_warnings);
    double err = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(back[i].face == pts[i].face);
        err = std::max(err, (back[i].bary - pts[i].bary).cwiseAbs().maxCoeff());
    }
    CHECK(map.build_warnings().total() == 0);
    CHECK(back_warnings.total() == 0);
    CHECK(err < 1e-8);
}

TEST_CASE(
    "tetwild-surface-history-transfers-onto-the-tet-mesh",
    "[tetwild][history]" + release_only)
{
    // Per-face labels on the input, carried to the tracked surface of the output and written into
    // the tet mesh tetwild wrote: its triangles on the tet mesh's own nodes, each with the label
    // of the input face it maps to.
    const fs::path dir = fs::current_path() / "tetwild_history_test";
    fs::create_directories(dir);
    Eigen::MatrixXd SV;
    Eigen::MatrixXi SF;
    REQUIRE(igl::readOBJ((fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj").string(), SV, SF));
    tracking::SurfaceData labelled;
    for (Eigen::Index v = 0; v < SV.rows(); ++v) labelled.vertices.push_back(SV.row(v).transpose());
    auto& label = labelled.face_fields["label"];
    for (Eigen::Index f = 0; f < SF.rows(); ++f) {
        labelled.faces.push_back({{uint64_t(SF(f, 0)), uint64_t(SF(f, 1)), uint64_t(SF(f, 2))}});
        label.push_back({double(f % 7)});
    }
    const fs::path input = dir / "sphere_labels.msh";
    tracking::write_surface_msh(input.string(), labelled);

    const fs::path out = dir / "labels";
    const fs::path hist = dir / "labels.wmtkhist";
    const auto exported = tetwild::tetwild_with_export({
        {"application", "tetwild"},
        {"input", {input.string()}},
        {"output", out.string()},
        {"history_output", hist.string()},
        {"max_iterations", 3},
        {"write_vtu", false},
    });
    const auto h = history::SurfaceHistory::load(hist.string());
    const tracking::SurfaceMap map(h);
    const auto in = tracking::read_surface({input.string()});
    std::string why;
    REQUIRE(tracking::matches_input(map, in, why));
    const auto transferred = tracking::transfer_fields(map, in);
    REQUIRE(transferred.faces.size() == size_t(exported.F.rows()));
    const fs::path with_labels = dir / "labels_final_with_labels.msh";
    tracking::write_surface_into_msh(
        out.string() + "_final.msh",
        with_labels.string(),
        transferred,
        tracking::output_rows(map, h.output_vertex_ids));

    const MeshData msh = io::read_mesh(with_labels);
    CHECK(msh.cell(CellType::Tetrahedron).connectivity.rows() == exported.T.rows());
    const CellData& triangles = msh.cell(CellType::Triangle);
    REQUIRE(size_t(triangles.connectivity.rows()) == transferred.faces.size());
    // Its vertices are the tet mesh's, where the surface has its vertices.
    size_t misplaced = 0, mislabelled = 0;
    for (size_t i = 0; i < transferred.faces.size(); ++i) {
        for (int k = 0; k < 3; ++k) {
            misplaced += msh.V.row(triangles.connectivity(i, k)).transpose() !=
                         transferred.vertices[transferred.faces[i][k]];
        }
    }
    const bool found = triangles.attributes.count("label") > 0;
    if (found) {
        const auto labels = triangles.attributes.at("label").doubles();
        const auto sources = triangles.attributes.at("input_face").integers();
        for (size_t i = 0; i < transferred.faces.size(); ++i) {
            REQUIRE(sources(i, 0) >= 0);
            mislabelled += labels(i, 0) != label[size_t(sources(i, 0))][0];
        }
    }
    CHECK(found);
    CHECK(misplaced == 0);
    CHECK(mislabelled == 0);
}
