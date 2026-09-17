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
    explicit File(const std::string& content) { std::ofstream(path) << content; }
    ~File()
    {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};
std::string replace(std::string s, const std::string& from, const std::string& to)
{
    s.replace(s.find(from), from.size(), to);
    return s;
}
} // namespace
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
    REQUIRE(result.vertices.rows() == 4);
    REQUIRE(result.tetrahedra.rows() == 1);
    REQUIRE(result.tetrahedra(0, 3) == 3);
    REQUIRE(result.vertices(3, 2) == -1);
    REQUIRE(result.source_vertex_ids == std::vector<int64_t>{10, 20, 30, 50});
    REQUIRE(result.vertex_tags == std::vector<int>{1, -1, -1, 2});
    REQUIRE(result.corr_input_vid == std::vector<int64_t>{-1, -1, -1, 10});
    REQUIRE(result.corr_input_vertex[3] == 0);
    REQUIRE(result.input_to_offset_vertices[0] == std::vector<size_t>{3});
    REQUIRE(result.offset_tet_tags == std::vector<int>{1});
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
    prism_main(data);
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
