#include <mshio/mshio.h>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <fstream>
#include <wmtk/components/prismatic_mesh/prismatic_mesh.hpp>

using namespace wmtk::components::prismatic_mesh;
namespace {
const std::string fixture = R"(<VTKFile type="UnstructuredGrid"><UnstructuredGrid>
<Piece NumberOfPoints="5" NumberOfCells="2">
<Points><DataArray type="Float64" NumberOfComponents="3" format="ascii">
0 0 0  1 0 0  0 1 0  0 0 1  0 0 -1
</DataArray></Points>
<Cells>
<DataArray type="Int32" Name="connectivity">0 1 2 3 0 2 1 4</DataArray>
<DataArray type="Int32" Name="offsets">4 8</DataArray>
<DataArray type="UInt8" Name="types">10 10</DataArray>
</Cells><PointData>
<DataArray type="Float64" Name="labels">1 0 0 2 2</DataArray>
<DataArray type="Float64" Name="vid">10 20 30 40 50</DataArray>
<DataArray type="Float64" Name="corr_input_vid">-1 -1 -1 10 10</DataArray>
</PointData><CellData>
<DataArray type="Float64" Name="tag_0">1 0</DataArray>
<DataArray type="Float64" Name="offset">0 0</DataArray>
<DataArray type="Float64" Name="offset_tag">0 1</DataArray>
</CellData></Piece></UnstructuredGrid></VTKFile>)";
struct File
{
    std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("prismatic_test_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".vtu");
    explicit File(const std::string& content, const std::string& extension = ".vtu")
    {
        path.replace_extension(extension);
        std::ofstream(path) << content;
    }
    ~File()
    {
        std::error_code ec;
        std::filesystem::remove(path, ec);
        std::filesystem::remove(
            path.parent_path() / (path.stem().string() + "_offset_faces.vtu"),
            ec);
        for (const auto* suffix : {".msh", ".vtu", "_offset_faces.vtu", "_hybrid.json"})
            std::filesystem::remove(
                path.parent_path() / (path.stem().string() + "_input_shell" + suffix),
                ec);
    }
};
std::string replace(std::string s, const std::string& from, const std::string& to)
{
    s.replace(s.find(from), from.size(), to);
    return s;
}
} // namespace
TEST_CASE("input and shell export option writes both formats alongside the main result")
{
    File file(fixture);
    File output("");
    prismatic_mesh(
        nlohmann::json{
            {"application", "prismatic_mesh"},
            {"input", file.path.string()},
            {"output", output.path.string()},
            {"iterations", 0},
            {"export_input_and_shell", true}});
    const auto stem = output.path.parent_path() / (output.path.stem().string() + "_input_shell");
    const auto full = load_prismatic_mesh(output.path);
    const auto subset = load_prismatic_mesh(stem.string() + ".vtu");
    const auto msh = mshio::load_msh(stem.string() + ".msh");
    REQUIRE(subset.vertices == full.vertices);
    REQUIRE(subset.tetrahedra == full.tetrahedra);
    REQUIRE(subset.source_vertex_ids == full.source_vertex_ids);
    REQUIRE(subset.corr_input_vid == full.corr_input_vid);
    REQUIRE(msh.elements.num_elements == 2);
    REQUIRE(msh.physical_groups.size() == 2);
}

TEST_CASE("MSH output retains isolated input vertices and compacts unused other vertices")
{
    auto content = replace(fixture, "-1 -1 -1 10 10", "-1 -1 -1 -1 10");
    content = replace(content, "Name=\"tag_0\">1 0", "Name=\"tag_0\">0 0");
    size_t expected_nodes = 5;
    SECTION("isolated input is retained")
    {
        content = replace(content, "1 0 0 2 2", "1 0 0 1 2");
    }
    SECTION("unused other vertex is removed")
    {
        content = replace(content, "1 0 0 2 2", "1 0 0 0 2");
        expected_nodes = 4;
    }
    File file(content);
    File output("", ".msh");
    REQUIRE_NOTHROW(prismatic_mesh(
        nlohmann::json{
            {"application", "prismatic_mesh"},
            {"input", file.path.string()},
            {"output", output.path.string()},
            {"iterations", 0}}));
    const auto msh = mshio::load_msh(output.path.string());
    REQUIRE_NOTHROW(mshio::validate_spec(msh));
    REQUIRE(msh.nodes.num_nodes == expected_nodes);
    REQUIRE(msh.elements.num_elements == 1);
    REQUIRE(msh.elements.entity_blocks.at(0).element_type == 4);
    const auto ids = std::find_if(msh.node_data.begin(), msh.node_data.end(), [](const auto& f) {
        return f.header.string_tags.at(0) == "vid";
    });
    REQUIRE(ids != msh.node_data.end());
    REQUIRE(ids->entries.back().data[0] == 50);
    const std::array<int64_t, 4> expected_ids = {10, 30, 20, 50};
    for (size_t j = 0; j < 4; ++j) {
        const auto tag = msh.elements.entity_blocks.at(0).data.at(j + 1);
        REQUIRE(ids->entries.at(tag - 1).data[0] == expected_ids[j]);
    }
}

TEST_CASE("prismatic mesh loads topology and resolves source IDs independently of row indices")
{
    File file(fixture);
    auto data = load_prismatic_mesh(file.path);
    REQUIRE(data.mesh->get_vertices().size() == 5);
    REQUIRE(data.mesh->get_tets().size() == 2);
    REQUIRE(data.vertices(4, 2) == -1);
    REQUIRE(data.tetrahedra(1, 1) == 2);
    REQUIRE(data.input_vertices == std::vector<size_t>{0});
    REQUIRE(data.offset_vertices == std::vector<size_t>{3, 4});
    REQUIRE(data.vertex_tags == std::vector<int>{1, -1, -1, 2, 2});
    REQUIRE(data.corr_input_vid[3] == 10);
    REQUIRE(data.corr_input_vertex[3] == 0);
    REQUIRE(data.input_to_offset_vertices[0] == std::vector<size_t>{3, 4});
    REQUIRE(data.input_cells == std::vector<int>{1, 0});
    REQUIRE(data.offset_tet_tags == std::vector<int>{-1, 1});
    File output("");
    auto output_stem = output.path.filename();
    output_stem.replace_extension();
    REQUIRE_NOTHROW(prismatic_mesh(
        nlohmann::json{
            {"application", "prismatic_mesh"},
            {"input", file.path.filename().string()},
            {"output", output_stem.string()},
            {"input_dir", file.path.parent_path().string()}}));
    auto result = load_prismatic_mesh(output.path);
    REQUIRE(result.vertices == data.vertices);
    REQUIRE(result.tetrahedra.rows() == 2);
    REQUIRE(result.tetrahedra.row(0) == data.tetrahedra.row(1));
    REQUIRE(result.tetrahedra.row(1) == data.tetrahedra.row(0));
    REQUIRE(result.source_vertex_ids == data.source_vertex_ids);
    REQUIRE(result.vertex_tags == data.vertex_tags);
    REQUIRE(result.corr_input_vid == data.corr_input_vid);
    REQUIRE(result.corr_input_vertex[3] == 0);
    REQUIRE(result.input_to_offset_vertices[0] == std::vector<size_t>{3, 4});
    REQUIRE(result.input_cells == std::vector<int>{0, 1});
    REQUIRE(result.offset_tet_tags == std::vector<int>{1, -1});
}
TEST_CASE("keeping the offset band filters topology and preserves vertex tags and correspondence")
{
    File file(fixture);
    auto data = load_prismatic_mesh(file.path);
    const wmtk::MatrixXd vertices = data.vertices;
    const wmtk::MatrixXi expected_tet = data.tetrahedra.bottomRows(1);
    const auto tags = data.vertex_tags;
    const auto ids = data.source_vertex_ids;
    const auto corr_ids = data.corr_input_vid;
    const auto corr_rows = data.corr_input_vertex;
    const auto reverse = data.input_to_offset_vertices;
    keep_offset_band(data);
    REQUIRE(data.tetrahedra == expected_tet);
    REQUIRE(data.mesh->get_tets().size() == 1);
    REQUIRE(data.mesh->get_vertices().size() == 4);
    for (const auto& v : data.mesh->get_vertices()) {
        REQUIRE(v.is_valid(*data.mesh));
        REQUIRE(v.vid(*data.mesh) != 3); // This vertex belonged only to the discarded tet.
    }
    REQUIRE(data.vertices == vertices);
    REQUIRE(data.vertex_tags == tags);
    REQUIRE(data.source_vertex_ids == ids);
    REQUIRE(data.corr_input_vid == corr_ids);
    REQUIRE(data.corr_input_vertex == corr_rows);
    REQUIRE(data.input_to_offset_vertices == reverse);
    REQUIRE(data.input_cells == std::vector<int>{0});
    REQUIRE(data.offset_tet_tags == std::vector<int>{1});
    keep_offset_band(data);
    REQUIRE(data.tetrahedra == expected_tet);
    REQUIRE(data.mesh->get_tets().size() == 1);
}

TEST_CASE("background retention constrains optimization while keeping the same shell targets")
{
    // One input tet above z=0, a shell apex at z=-1, and an exterior tet extending
    // to z=-2, followed by a fixed exterior tet with no offset vertex. A target below
    // z=-2 is valid for the shell but would invert the adjacent exterior tet.
    File file(R"(<VTKFile type="UnstructuredGrid"><UnstructuredGrid>
<Piece NumberOfPoints="7" NumberOfCells="4">
<Points><DataArray type="Float64" NumberOfComponents="3" format="ascii">
0 0 0  1 0 0  0 1 0  0 0 -1  0 0 1  0 0 -2  0 0 -3
</DataArray></Points><Cells>
<DataArray type="Int32" Name="connectivity">0 1 2 4 0 2 1 3 2 1 3 5 2 1 5 6</DataArray>
<DataArray type="Int32" Name="offsets">4 8 12 16</DataArray>
<DataArray type="UInt8" Name="types">10 10 10 10</DataArray>
</Cells><PointData>
<DataArray type="Float64" Name="labels">1 1 1 2 1 0 0</DataArray>
<DataArray type="Float64" Name="vid">100 200 300 400 500 600 700</DataArray>
<DataArray type="Float64" Name="corr_input_vid">-1 -1 -1 100 -1 -1 -1</DataArray>
</PointData><CellData>
<DataArray type="Float64" Name="tag_0">1 0 0 0</DataArray>
<DataArray type="Float64" Name="offset_tag">0 1 0 0</DataArray>
</CellData></Piece></UnstructuredGrid></VTKFile>)");
    auto shell = load_prismatic_mesh(file.path);
    auto background = load_prismatic_mesh(file.path);
    const auto original = background.vertices;
    OptimizationOptions options;
    options.iterations = 1;
    SECTION("background blocks an otherwise valid full target step") {}
    SECTION("zero iterations retains background without moving vertices")
    {
        options.iterations = 0;
    }
    SECTION("fixed background is still subject to initial volume validation")
    {
        background.vertices(6, 2) = -2; // Degenerate, outside every offset vertex's star.
        options.keep_background_mesh = true;
        REQUIRE_THROWS(prism_main(background, 3, options));
        REQUIRE(background.vertices.row(3) == original.row(3));
        return;
    }
    SECTION("small positive fixed background does not abort optimization or hybrid export")
    {
        background.vertices(6, 2) = -2 - 1e-6;
        const auto fixed_position = background.vertices.row(6).eval();
        options.keep_background_mesh = true;
        options.min_tet_volume = 0.01;
        REQUIRE_NOTHROW(prism_main(background, 3, options));
        REQUIRE(background.vertices.row(6) == fixed_position);
        REQUIRE(background.optimization_iterations[0].smoothed_vertices == 1);
        REQUIRE(tet_volume_above_threshold(background.vertices, {2, 1, 5, 6}, 0));
        REQUIRE_FALSE(
            tet_volume_above_threshold(background.vertices, {2, 1, 5, 6}, options.min_tet_volume));
        REQUIRE_NOTHROW(build_prism_dominant_mesh(background, options.min_tet_volume));
        return;
    }
    prism_main(shell, 3, options);
    options.keep_background_mesh = true;
    prism_main(background, 3, options);
    REQUIRE(shell.tetrahedra.rows() == 2);
    REQUIRE(background.tetrahedra.rows() == 4);
    REQUIRE(background.input_average_edge_length == shell.input_average_edge_length);
    REQUIRE(background.target_positions == shell.target_positions);
    REQUIRE(background.optimal_normals == shell.optimal_normals);
    REQUIRE(background.vertex_component_ids == shell.vertex_component_ids);
    REQUIRE(background.singular_vertex_tags == shell.singular_vertex_tags);
    REQUIRE(background.input_cells == std::vector<int>{0, 0, 0, 1});
    REQUIRE(background.offset_tet_tags == std::vector<int>{1, -1, -1, -1});
    REQUIRE(background.tetrahedra.row(2) == wmtk::Vector4i(2, 1, 5, 6).transpose());
    for (size_t v : {0, 1, 2, 4, 5, 6}) REQUIRE(background.vertices.row(v) == original.row(v));
    REQUIRE(background.fixed_background_vertices.empty());
    if (options.iterations > 0) {
        REQUIRE(shell.vertices.row(3) == shell.target_positions.row(3));
        REQUIRE_FALSE(tet_volume_above_threshold(shell.vertices, {2, 1, 3, 5}, 0));
        const wmtk::Vector3d expected = 0.75 * original.row(3).transpose() +
                                        0.25 * background.target_positions.row(3).transpose();
        REQUIRE((background.vertices.row(3).transpose() - expected).norm() < 1e-12);
    } else {
        REQUIRE(background.vertices == original);
    }
    REQUIRE(background.mesh->check_mesh_connectivity_validity());
    for (const auto& t : background.mesh->get_tets())
        REQUIRE(tet_volume_above_threshold(
            background.vertices,
            background.mesh->oriented_tet_vids(t),
            options.min_tet_volume));
    const auto [interface, fid] = background.mesh->tuple_from_face(std::array<size_t, 3>{1, 2, 3});
    REQUIRE_FALSE(interface.is_boundary_face(*background.mesh));
    const auto [fixed_interface, fixed_fid] =
        background.mesh->tuple_from_face(std::array<size_t, 3>{1, 2, 5});
    REQUIRE_FALSE(fixed_interface.is_boundary_face(*background.mesh));
    // Exercise JSON validation, option propagation, and export of background vertices/cells.
    File output("");
    REQUIRE_NOTHROW(prismatic_mesh(
        nlohmann::json{
            {"application", "prismatic_mesh"},
            {"input", file.path.string()},
            {"output", output.path.string()},
            {"thicknessratio", 3.0},
            {"iterations", options.iterations},
            {"keep_background_mesh", true}}));
    auto exported = load_prismatic_mesh(output.path);
    REQUIRE(exported.vertices == background.vertices);
    REQUIRE(exported.tetrahedra == background.tetrahedra);
    REQUIRE(exported.vertex_tags == background.vertex_tags);
    REQUIRE(exported.source_vertex_ids == background.source_vertex_ids);
    REQUIRE(exported.corr_input_vid == background.corr_input_vid);
    REQUIRE(exported.input_cells == background.input_cells);
    REQUIRE(exported.offset_tet_tags == background.offset_tet_tags);
}

TEST_CASE("keeping an empty offset band produces an empty active mesh")
{
    File file(replace(fixture, "Name=\"offset_tag\">0 1", "Name=\"offset_tag\">0 0"));
    auto data = load_prismatic_mesh(file.path);
    keep_offset_band(data);
    REQUIRE(data.tetrahedra.rows() == 0);
    REQUIRE(data.mesh->get_tets().empty());
    REQUIRE(data.mesh->get_vertices().empty());
    REQUIRE(data.offset_tet_tags.empty());
    REQUIRE(data.input_cells.empty());
    REQUIRE(data.vertex_tags == std::vector<int>{1, -1, -1, 2, 2});
}

TEST_CASE("prism pipeline restores input volume and exports all input vertices")
{
    // A subdivided input tet (0..4), one shell tet with offset apex 5, an exterior
    // ambient tet with apex 6, and an isolated input point 7. IDs differ from row indices.
    File file(R"(<VTKFile type="UnstructuredGrid"><UnstructuredGrid>
<Piece NumberOfPoints="8" NumberOfCells="6">
<Points><DataArray type="Float64" NumberOfComponents="3" format="ascii">
0 0 0  1 0 0  0 1 0  0 0 1  0.25 0.25 0.25  0 0 -1  0 0 -2  4 4 4
</DataArray></Points><Cells>
<DataArray type="Int32" Name="connectivity">4 1 2 3 0 4 2 3 0 1 4 3 0 1 2 4 0 2 1 5 2 1 5 6</DataArray>
<DataArray type="Int32" Name="offsets">4 8 12 16 20 24</DataArray>
<DataArray type="UInt8" Name="types">10 10 10 10 10 10</DataArray>
</Cells><PointData>
<DataArray type="Float64" Name="labels">1 1 1 1 1 2 0 1</DataArray>
<DataArray type="Float64" Name="vid">100 200 300 400 500 600 700 800</DataArray>
<DataArray type="Float64" Name="corr_input_vid">-1 -1 -1 -1 -1 100 -1 -1</DataArray>
</PointData><CellData>
<DataArray type="Float64" Name="tag_0">1 1 1 1 0 0</DataArray>
<DataArray type="Float64" Name="offset_tag">0 0 0 0 1 0</DataArray>
</CellData></Piece></UnstructuredGrid></VTKFile>)");
    auto data = load_prismatic_mesh(file.path);
    const auto original_vertices = data.vertices;
    const wmtk::MatrixXi original_volume = data.tetrahedra.topRows(4);
    auto band = load_prismatic_mesh(file.path);
    OptimizationOptions options;
    options.iterations = 1;
    SECTION("with smoothing") {}
    SECTION("without optimization")
    {
        options.iterations = 0;
    }
    SECTION("empty band")
    {
        data.offset_tet_tags.assign(6, -1);
        band.offset_tet_tags = data.offset_tet_tags;
    }
    keep_offset_band(band);
    evaluate_target_positions(band, 0.1);
    optimize_prismatic_mesh(band, options);
    prism_main(data, 0.1, options);
    REQUIRE(data.tetrahedra.rows() == 4 + band.tetrahedra.rows());
    REQUIRE(data.tetrahedra.bottomRows(4) == original_volume);
    REQUIRE(data.tetrahedra.topRows(band.tetrahedra.rows()) == band.tetrahedra);
    REQUIRE(data.vertices == band.vertices);
    REQUIRE(data.input_average_edge_length == band.input_average_edge_length);
    REQUIRE(data.target_positions == band.target_positions);
    REQUIRE(data.vertex_component_ids == band.vertex_component_ids);
    for (size_t v : data.input_vertices) REQUIRE(data.vertices.row(v) == original_vertices.row(v));
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
    for (const auto& t : data.mesh->get_tets()) {
        const size_t tid = t.tid(*data.mesh);
        const bool in_volume = tid >= static_cast<size_t>(band.tetrahedra.rows());
        REQUIRE(data.input_cells[tid] == (in_volume ? 1 : 0));
        REQUIRE(data.offset_tet_tags[tid] == (in_volume ? -1 : 1));
        REQUIRE(tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(t), 0));
    }
    if (band.tetrahedra.rows() > 0) {
        // The input triangle is a shared interior face after reattachment, not duplicated.
        const auto [interface, fid] = data.mesh->tuple_from_face(std::array<size_t, 3>{0, 1, 2});
        REQUIRE_FALSE(interface.is_boundary_face(*data.mesh));
        REQUIRE(interface.switch_tetrahedron(*data.mesh));
        if (options.iterations > 0) REQUIRE(data.vertices.row(5) != original_vertices.row(5));
    }
    File output("");
    write_prismatic_mesh(data, output.path);
    auto result = load_prismatic_mesh(output.path);
    REQUIRE(result.input_vertices.size() == 6); // includes interior and isolated input points
    REQUIRE(result.tetrahedra.rows() == data.tetrahedra.rows());
    REQUIRE(result.input_cells == data.input_cells);
    REQUIRE(result.offset_tet_tags == data.offset_tet_tags);
    REQUIRE(
        std::find(result.source_vertex_ids.begin(), result.source_vertex_ids.end(), 700) ==
        result.source_vertex_ids.end()); // exterior ambient point is omitted
    for (size_t v : data.input_vertices) {
        const auto found = std::find(
            result.source_vertex_ids.begin(),
            result.source_vertex_ids.end(),
            data.source_vertex_ids[v]);
        REQUIRE(found != result.source_vertex_ids.end());
        const size_t row = std::distance(result.source_vertex_ids.begin(), found);
        REQUIRE(result.vertices.row(row) == original_vertices.row(v));
        REQUIRE(result.vertex_tags[row] == 1);
    }
    // Running the pipeline again must not duplicate restored input cells.
    const auto cells = data.tetrahedra;
    options.iterations = 0;
    prism_main(data, 0.1, options);
    REQUIRE(data.tetrahedra == cells);
}

TEST_CASE(
    "offset faces are classified by correspondence equality and shared faces are counted once")
{
    File file(fixture);
    auto data = load_prismatic_mesh(file.path);
    // The two tetrahedra share face {0,1,2}. Classify it even though it is not a boundary face:
    // the definition only requires three offset vertices.
    data.vertex_tags = {2, 2, 2, 1, -1};
    int expected = 1;
    SECTION("three different")
    {
        data.corr_input_vid = {10, 20, 30, -1, -1};
    }
    SECTION("first two equal")
    {
        data.corr_input_vid = {10, 10, 30, -1, -1};
        expected = 2;
    }
    SECTION("last two equal")
    {
        data.corr_input_vid = {10, 30, 30, -1, -1};
        expected = 2;
    }
    SECTION("first and last equal")
    {
        data.corr_input_vid = {10, 30, 10, -1, -1};
        expected = 2;
    }
    SECTION("all equal")
    {
        data.corr_input_vid = {10, 10, 10, -1, -1};
        expected = 3;
    }
    label_offset_faces(data);
    size_t offset_count = 0;
    for (const auto& face : data.mesh->get_faces()) {
        const int tag = data.offset_face_tags.at(face.fid(*data.mesh));
        if (tag == -1) continue;
        REQUIRE(tag == expected);
        REQUIRE_FALSE(face.is_boundary_face(*data.mesh));
        ++offset_count;
    }
    REQUIRE(offset_count == 1);
    data.vertex_tags[0] = 1;
    label_offset_faces(data);
    for (const auto& face : data.mesh->get_faces()) {
        REQUIRE(data.offset_face_tags.at(face.fid(*data.mesh)) == -1);
    }
}

TEST_CASE("offset face labeling handles missing correspondence and empty meshes")
{
    File file(fixture);
    auto data = load_prismatic_mesh(file.path);
    data.vertex_tags = {2, 2, 2, 1, -1};
    REQUIRE_THROWS(label_offset_faces(data));
    data.offset_tet_tags = {-1, -1};
    keep_offset_band(data);
    label_offset_faces(data);
    REQUIRE(data.offset_face_tags.empty());
}

TEST_CASE("prismatic mesh rejects invalid correspondence and mesh arrays")
{
    std::string content;
    SECTION("missing array")
    {
        content = replace(fixture, "corr_input_vid", "missing");
    }
    SECTION("unknown target")
    {
        content = replace(fixture, "-1 -1 -1 10 10", "-1 -1 -1 99 10");
    }
    SECTION("non-input target")
    {
        content = replace(fixture, "-1 -1 -1 10 10", "-1 -1 -1 20 10");
    }
    SECTION("offset without correspondence")
    {
        content = replace(fixture, "-1 -1 -1 10 10", "-1 -1 -1 -1 10");
    }
    SECTION("duplicate IDs")
    {
        content = replace(fixture, "10 20 30 40 50", "10 10 30 40 50");
    }
    SECTION("fractional ID")
    {
        content = replace(fixture, "10 20 30 40 50", "10 20.5 30 40 50");
    }
    SECTION("invalid connectivity")
    {
        content = replace(fixture, "0 1 2 3 0 2 1 4", "0 1 2 3 0 2 1 9");
    }
    SECTION("wrong cell type")
    {
        content = replace(fixture, ">10 10<", ">10 12<");
    }
    SECTION("short array")
    {
        content = replace(fixture, "1 0 0 2 2", "1 0 0 2");
    }
    SECTION("compressed")
    {
        content = replace(fixture, "<VTKFile ", "<VTKFile compressor=\"vtkZLibDataCompressor\" ");
    }
    SECTION("bad base64")
    {
        content = replace(fixture, "format=\"ascii\"", "format=\"binary\"");
    }
    File file(content);
    REQUIRE_THROWS(load_prismatic_mesh(file.path));
}
TEST_CASE("prismatic mesh requires an input and the correct application")
{
    REQUIRE_THROWS(prismatic_mesh(nlohmann::json{{"application", "prismatic_mesh"}}));
    REQUIRE_THROWS(prismatic_mesh(
        nlohmann::json{
            {"application", "other"},
            {"input", "x.vtu"},
            {"output", "resultmesh.vtu"}}));
    REQUIRE_THROWS(
        prismatic_mesh(nlohmann::json{{"application", "prismatic_mesh"}, {"input", "x.vtu"}}));
    REQUIRE_THROWS(load_prismatic_mesh("missing_mesh.vtu"));
}

TEST_CASE("prismatic mesh reads inline binary little endian Float64 UInt64")
{
    auto content = replace(
        fixture,
        R"(<VTKFile type="UnstructuredGrid">)",
        R"(<VTKFile type="UnstructuredGrid" byte_order="LittleEndian" header_type="UInt64">)");
    const auto begin = content.find("<Points>");
    const auto end = content.find("</Points>") + std::string("</Points>").size();
    content.replace(
        begin,
        end - begin,
        R"(<Points><DataArray type="Float64" NumberOfComponents="3" format="binary">eAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAADwPwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAPA/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA8D8AAAAAAAAAAAAAAAAAAAAAAAAAAAAA8L8=</DataArray></Points>)");
    File file(content);
    auto data = load_prismatic_mesh(file.path);
    REQUIRE(data.vertices(4, 2) == -1);
    REQUIRE(data.vertices(1, 0) == 1);
    REQUIRE(data.corr_input_vertex[4] == 0);
}

TEST_CASE("prismatic mesh reads inline binary big endian Float32 UInt32")
{
    auto content = replace(
        fixture,
        R"(<VTKFile type="UnstructuredGrid">)",
        R"(<VTKFile type="UnstructuredGrid" byte_order="BigEndian" header_type="UInt32">)");
    const auto begin = content.find("<Points>");
    const auto end = content.find("</Points>") + std::string("</Points>").size();
    content.replace(
        begin,
        end - begin,
        R"(<Points><DataArray type="Float32" NumberOfComponents="3" format="binary">AAAAPAAAAAAAAAAAAAAAAD+AAAAAAAAAAAAAAAAAAAA/gAAAAAAAAAAAAAAAAAAAP4AAAAAAAAAAAAAAv4AAAA==</DataArray></Points>)");
    File file(content);
    auto data = load_prismatic_mesh(file.path);
    REQUIRE(data.vertices(4, 2) == -1);
    REQUIRE(data.vertices(1, 0) == 1);
    REQUIRE(data.corr_input_vertex[4] == 0);
}
