// The fields of a surface, through the toolkit's mesh files: what io::write_mesh and
// wm.write_mesh write, isotropic_remeshing's feature_fields and tracking's transfer read, and
// what transfer writes, io::read_mesh and wm.read_mesh read -- per-face fields as triangle
// attributes, per-corner ones (uv, normal) as triangle attributes of three values per corner.

#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/components/tracking/surface_io.hpp>
#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/history/SurfaceHistory.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <map>
#include <set>

namespace fs = std::filesystem;
using namespace wmtk;
using namespace wmtk::components;

namespace {

/// The sphere as io::read_mesh reads it -- uv on every corner, six columns -- with a label per
/// face, `side` (Float64) and `octant` (Int64).
MeshData labelled_sphere()
{
    MeshData m = io::read_mesh(fs::path(WMTK_DATA_DIR) / "models" / "sphere.obj");
    auto& triangles = m.cells.at(CellType::Triangle);
    const auto& F = triangles.connectivity;
    DataMatrix<double> side(F.rows(), 1);
    DataMatrix<int64_t> octant(F.rows(), 1);
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        const Eigen::RowVector3d c = (m.V.row(F(f, 0)) + m.V.row(F(f, 1)) + m.V.row(F(f, 2))) / 3;
        side(f, 0) = c.x() > 0 ? 1. : 0.;
        octant(f, 0) = (c.x() > 0) + 2 * (c.y() > 0) + 4 * (c.z() > 0);
    }
    triangles.attributes["side"].values = side;
    triangles.attributes["octant"].values = octant;
    return m;
}

/// The edges between two faces of `m` that differ in `side`, or disagree on `uv` at either end.
std::set<std::array<uint64_t, 2>> boundaries(const MeshData& m)
{
    const auto& t = m.cells.at(CellType::Triangle);
    const auto& F = t.connectivity;
    const auto side = t.attributes.at("side").doubles();
    const auto uv = t.attributes.at("uv").doubles();
    const auto at = [&](Eigen::Index f, int64_t v) {
        for (int c = 0; c < 3; ++c) {
            if (F(f, c) == v) return Eigen::Vector2d(uv(f, 2 * c), uv(f, 2 * c + 1));
        }
        return Eigen::Vector2d(NAN, NAN);
    };
    std::map<std::array<uint64_t, 2>, std::vector<Eigen::Index>> edge_faces;
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            uint64_t a = F(f, k), b = F(f, (k + 1) % 3);
            if (a > b) std::swap(a, b);
            edge_faces[{{a, b}}].push_back(f);
        }
    }
    std::set<std::array<uint64_t, 2>> out;
    for (const auto& [e, fs] : edge_faces) {
        if (fs.size() != 2) continue;
        const auto f = fs[0], g = fs[1];
        if (side(f, 0) != side(g, 0) || at(f, e[0]) != at(g, e[0]) || at(f, e[1]) != at(g, e[1])) {
            out.insert(e);
        }
    }
    return out;
}

} // namespace

TEST_CASE("surface-io-fields-through-mesh-files", "[surface_map][io]")
{
    const fs::path dir = fs::current_path() / "surface_io_fields";
    fs::create_directories(dir);
    const MeshData sphere = labelled_sphere();
    const auto& in_triangles = sphere.cells.at(CellType::Triangle);
    REQUIRE(in_triangles.attributes.at("uv").cols() == 6);
    const fs::path input = dir / "sphere.msh";
    io::write_mesh(input, sphere);

    // 1. A file io::write_mesh wrote is read as a surface: uv per corner, the labels per face,
    // their types kept; written again, it reads back as it was.
    const tracking::SurfaceData s = tracking::read_surface_msh(input.string());
    REQUIRE(s.faces.size() == size_t(in_triangles.connectivity.rows()));
    REQUIRE(s.corner_fields.count("uv"));
    REQUIRE(s.face_fields.count("side"));
    REQUIRE(s.face_fields.count("octant"));
    CHECK(s.field_types.at("octant").type == 1);
    const auto uv = in_triangles.attributes.at("uv").doubles();
    for (size_t f = 0; f < s.faces.size(); ++f) {
        for (int k = 0; k < 6; ++k) REQUIRE(s.corner_fields.at("uv")[f][k] == uv(f, k));
    }
    tracking::write_surface_msh((dir / "again.msh").string(), s);
    const MeshData again = io::read_mesh(dir / "again.msh");
    const auto& again_triangles = again.cells.at(CellType::Triangle);
    CHECK(again_triangles.connectivity == in_triangles.connectivity);
    CHECK(again_triangles.attributes.at("uv").doubles() == uv);
    CHECK(again_triangles.attributes.at("octant").values.index() == 1);
    CHECK(
        again_triangles.attributes.at("octant").integers() ==
        in_triangles.attributes.at("octant").integers());

    // 2. feature_fields reads the label and the texture seams from that file and keeps them as
    // curves, which the history starts with.
    const fs::path output = dir / "remeshed.obj";
    const fs::path tracking_file = dir / "remeshed.wmtkhist";
    isotropic_remeshing::isotropic_remeshing({
        {"application", "isotropic_remeshing"},
        {"input", input.string()},
        {"output", output.string()},
        {"feature_fields", {"side", "uv"}},
        {"history_output", tracking_file.string()},
        {"length_rel", 0.08},
        {"max_iterations", 2},
    });
    const auto h = history::SurfaceHistory::load(tracking_file.string());
    CHECK(nlohmann::json::parse(h.metadata).at("route") == "file");
    const auto expected = boundaries(sphere);
    REQUIRE(!expected.empty());
    CHECK(
        std::set<std::array<uint64_t, 2>>(h.feature_edges.begin(), h.feature_edges.end()) ==
        expected);

    // 3. transfer writes the output with every field, as io::read_mesh reads it: the labels of
    // their types, uv of six columns, input_face Int64; and the textured OBJ.
    const fs::path transferred = dir / "transferred.msh";
    const fs::path textured = dir / "transferred.obj";
    tracking::tracking({
        {"application", "tracking"},
        {"history", tracking_file.string()},
        {"operation", "transfer"},
        {"output", transferred.string()},
        {"output_obj", textured.string()},
    });
    const MeshData out = io::read_mesh(transferred);
    const MeshData remeshed = io::read_mesh(output);
    const auto& t = out.cells.at(CellType::Triangle);
    REQUIRE(t.connectivity.rows() == remeshed.cell(CellType::Triangle).connectivity.rows());
    CHECK(t.attributes.at("side").values.index() == 0);
    CHECK(t.attributes.at("octant").values.index() == 1);
    CHECK(t.attributes.at("input_face").values.index() == 1);
    CHECK(t.attributes.at("uv").cols() == 6);
    const auto from = t.attributes.at("input_face").integers();
    const auto octant = t.attributes.at("octant").integers();
    const auto in_octant = in_triangles.attributes.at("octant").integers();
    size_t mislabelled = 0;
    for (Eigen::Index f = 0; f < t.connectivity.rows(); ++f) {
        REQUIRE(from(f, 0) >= 0);
        mislabelled += octant(f, 0) != in_octant(from(f, 0), 0);
    }
    CHECK(mislabelled == 0);
    const MeshData obj = io::read_mesh(textured);
    REQUIRE(obj.cells.at(CellType::Triangle).attributes.count("uv"));
    const auto obj_uv = obj.cells.at(CellType::Triangle).attributes.at("uv").doubles();
    CHECK((obj_uv - t.attributes.at("uv").doubles()).cwiseAbs().maxCoeff() < 1e-12);
    fs::remove_all(dir);
}

TEST_CASE("surface-io-reads-standard-gmsh-element-data", "[surface_map][io]")
{
    // A gmsh file as gmsh writes one: two triangles, a label for each as $ElementData. It is
    // carried as a field of the faces, as io::read_mesh reads it.
    const fs::path dir = fs::current_path() / "surface_io_gmsh";
    fs::create_directories(dir);
    const fs::path path = dir / "labelled.msh";
    {
        std::ofstream f(path);
        f << "$MeshFormat\n2.2 0 8\n$EndMeshFormat\n"
          << "$Nodes\n4\n1 0 0 0\n2 1 0 0\n3 0 1 0\n4 1 1 0\n$EndNodes\n"
          << "$Elements\n2\n1 2 2 1 1 1 2 3\n2 2 2 2 2 2 4 3\n$EndElements\n"
          << "$ElementData\n1\n\"label\"\n1\n0.0\n3\n0\n1\n2\n1 5\n2 7\n$EndElementData\n";
    }
    const tracking::SurfaceData s = tracking::read_surface_msh(path.string());
    REQUIRE(s.faces.size() == 2);
    REQUIRE(s.face_fields.count("label"));
    CHECK(s.face_fields.at("label") == std::vector<std::vector<double>>{{5.}, {7.}});
    fs::remove_all(dir);
}
