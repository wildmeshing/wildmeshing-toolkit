#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/MeshData.hpp>
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/DataWriter.hpp>
#include <wmtk/io/mesh_io.hpp>

using namespace wmtk;
namespace {
struct TempMesh
{
    std::filesystem::path path;
    explicit TempMesh(const std::string& ext)
        : path(
              std::filesystem::temp_directory_path() /
              std::filesystem::path(
                  "wmtk_mesh_data_test_" +
                  std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
                  ext))
    {}
    ~TempMesh()
    {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};
MeshData example()
{
    MeshData m;
    m.V.resize(7, 3);
    m.V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 3, 3, 3;
    m.cells[CellType::Point].connectivity.resize(1, 1);
    m.cells[CellType::Point].connectivity << 6;
    m.cells[CellType::Edge].connectivity.resize(1, 2);
    m.cells[CellType::Edge].connectivity << 1, 0;
    m.cells[CellType::Triangle].connectivity.resize(2, 3);
    m.cells[CellType::Triangle].connectivity << 0, 1, 2, 0, 2, 3;
    m.cells[CellType::Quad].connectivity.resize(1, 4);
    m.cells[CellType::Quad].connectivity << 0, 1, 4, 3;
    m.cells[CellType::Tetrahedron].connectivity.resize(1, 4);
    m.cells[CellType::Tetrahedron].connectivity << 0, 1, 2, 3;
    m.cells[CellType::Prism].connectivity.resize(1, 6);
    m.cells[CellType::Prism].connectivity << 0, 1, 2, 3, 4, 5;
    m.cells[CellType::Pyramid].connectivity.resize(1, 5);
    m.cells[CellType::Pyramid].connectivity << 0, 1, 4, 3, 2;
    DataMatrix<int64_t> ids(7, 1);
    ids << -1, 4, 9, 12, 100, 200, 9007199254740991LL;
    m.vertex_attributes["source ID"].values = ids;
    DataMatrix<double> vector(7, 2);
    for (int i = 0; i < 7; ++i) vector.row(i) << 0.125 * i, -0.5 * i;
    m.vertex_attributes["two components"] = {vector, {"a", "b"}};
    DataMatrix<uint8_t> tags(2, 2);
    tags << 1, 1, 1, 0;
    m.cells[CellType::Triangle].attributes["tags"] = {tags, {"first group", "overlap"}};
    m.cells[CellType::Triangle].attributes["DOFs"].values =
        DataMatrix<double>::Constant(2, 36, 0.25).eval();
    m.cells[CellType::Edge].attributes["orientation"].values =
        DataMatrix<int64_t>::Constant(1, 1, -1).eval();
    return m;
}
void equal(const MeshData& a, const MeshData& b)
{
    REQUIRE(a.V == b.V);
    REQUIRE(a.cells.size() == b.cells.size());
    auto attrs = [](const MeshAttributes& a, const MeshAttributes& b) {
        REQUIRE(a.size() == b.size());
        for (const auto& [name, x] : a) {
            REQUIRE(b.count(name));
            const auto& y = b.at(name);
            REQUIRE(x.values.index() == y.values.index());
            REQUIRE(x.labels == y.labels);
            REQUIRE(x.doubles() == y.doubles());
        }
    };
    attrs(a.vertex_attributes, b.vertex_attributes);
    for (const auto& [type, c] : a.cells) {
        REQUIRE(b.cells.count(type));
        const auto& d = b.cells.at(type);
        REQUIRE(c.connectivity == d.connectivity);
        attrs(c.attributes, d.attributes);
    }
}
} // namespace
TEST_CASE("MeshData typed mixed mesh round trip", "[mesh_data][io]")
{
    const auto m = example();
    m.validate();
    TempMesh file(".msh");
    io::write_mesh(file.path, m);
    equal(m, io::read_mesh(file.path));
}
TEST_CASE("MeshData validates and rejects lossy I/O", "[mesh_data][io]")
{
    auto m = example();
    SECTION("invalid index")
    {
        m.cells[CellType::Edge].connectivity(0, 0) = 100;
        REQUIRE_THROWS(m.validate());
    }
    SECTION("wrong row count")
    {
        m.vertex_attributes["bad"].values = DataMatrix<double>::Zero(1, 1).eval();
        REQUIRE_THROWS(m.validate());
    }
    SECTION("missing labels")
    {
        m.cells[CellType::Triangle].attributes["tags"].labels = {"one"};
        REQUIRE_THROWS(m.validate());
    }
    SECTION("int64 precision")
    {
        m.vertex_attributes["source ID"].values =
            DataMatrix<int64_t>::Constant(7, 1, 9007199254740993LL).eval();
        TempMesh file(".msh");
        REQUIRE_THROWS(io::write_mesh(file.path, m));
    }
    SECTION("coordinate NaN")
    {
        m.V(0, 0) = std::numeric_limits<double>::quiet_NaN();
        REQUIRE_THROWS(m.validate());
    }
}
TEST_CASE("MeshData keeps degenerate cells for component cleaning", "[mesh_data][io]")
{
    auto m = example();
    m.cells[CellType::Triangle].connectivity(1, 2) = 0;
    REQUIRE_NOTHROW(m.validate());
    TempMesh file(".obj");
    {
        std::ofstream f(file.path);
        f << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\nf 2 2 3\n";
    }
    const auto read = io::read_mesh(file.path);
    REQUIRE(read.cell(CellType::Triangle).connectivity.rows() == 2);
}
TEST_CASE("MeshData reads OBJ attributes and orientation without cleaning", "[mesh_data][io]")
{
    TempMesh file(".obj");
    {
        std::ofstream f(file.path);
        f << "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 0\nvt 0 0\nvt 1 0\nvt 0 1\nvn 0 0 1\nf 1/1/1 2/2/1 "
             "3/3/1\nl 3 2 1\n";
    }
    const auto m = io::read_mesh(file.path);
    REQUIRE(m.V.rows() == 4);
    REQUIRE(m.cell(CellType::Triangle).attributes.at("uv").cols() == 6);
    REQUIRE(m.cell(CellType::Triangle).attributes.at("normal").cols() == 9);
    REQUIRE(m.cell(CellType::Edge).connectivity(0, 0) == 2);
    REQUIRE(m.cell(CellType::Edge).connectivity(1, 1) == 0);
}
TEST_CASE("OBJ corner fields need a value on every corner", "[mesh_data][io]")
{
    TempMesh file(".obj");
    SECTION("unreferenced vt and vn records")
    {
        std::ofstream(file.path) << "v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvn 0 0 1\nf 1 2 3\n";
    }
    SECTION("only some corners reference them")
    {
        std::ofstream(file.path) << "v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvn 0 0 1\n"
                                    "f 1/1/1 2 3/1/1\n";
    }
    const auto m = io::read_mesh(file.path);
    REQUIRE(m.cell(CellType::Triangle).connectivity.rows() == 1);
    REQUIRE(m.cell(CellType::Triangle).attributes.empty());
}
TEST_CASE("OBJ rejects out-of-range indices", "[mesh_data][io]")
{
    TempMesh file(".obj");
    SECTION("relative vertex")
    {
        std::ofstream(file.path) << "v 0 0 0\nv 1 0 0\nf -1 -2 -3\n";
    }
    SECTION("relative UV")
    {
        std::ofstream(file.path) << "v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nf 1/-2 2/1 3/1\n";
    }
    SECTION("UV")
    {
        std::ofstream(file.path) << "v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nf 1/2 2/1 3/1\n";
    }
    REQUIRE_THROWS(io::read_mesh(file.path));
}
TEST_CASE("Named input selection is strict and ordered", "[mesh_data][io]")
{
    MeshData a;
    a.V.resize(3, 3);
    a.V << 0, 0, 0, 1, 0, 0, 0, 1, 0;
    a.cells[CellType::Triangle].connectivity.resize(1, 3);
    a.cells[CellType::Triangle].connectivity << 0, 1, 2;
    MeshData b = a;
    b.V.array() += 2;
    MeshMap inputs{{"a", a}, {"b", b}, {"unselected", example()}};
    io::ComponentIO context(&inputs);
    MatrixXd V;
    MatrixXi F;
    std::vector<int> source;
    context.triangles(std::vector<std::string>{"b", "a"}, V, F, -1, -1, &source);
    REQUIRE(V.topRows(3) == b.V);
    REQUIRE(source == std::vector<int>{0, 1});
    REQUIRE_THROWS(context.triangles("missing", V, F));
    inputs.at("a").vertex_attributes["unexpected"].values = DataMatrix<double>::Zero(3, 1).eval();
    REQUIRE_THROWS(context.triangles("a", V, F));
}
TEST_CASE("Several memory inputs are cleaned like several files", "[mesh_data][io]")
{
    // An isolated vertex of the first input lies within the merge tolerance of a referenced
    // vertex of the second. Cleaning each input first would drop it before the merge.
    TempMesh first(".obj"), second(".obj");
    std::ofstream(first.path) << "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 5.0004 0 0\nf 1 2 3\n";
    std::ofstream(second.path) << "v 5 0 0\nv 6 0 0\nv 5 1 0\nf 1 2 3\n";
    MatrixXd V_file, V_memory;
    MatrixXi F_file, F_memory;
    std::vector<int> source_file, source_memory;
    io::read_triangle_mesh(
        {first.path.string(), second.path.string()},
        V_file,
        F_file,
        -1,
        1e-3,
        &source_file);
    MeshMap inputs{{"first", io::read_mesh(first.path)}, {"second", io::read_mesh(second.path)}};
    io::ComponentIO context(&inputs);
    context.triangles(
        std::vector<std::string>{"first", "second"},
        V_memory,
        F_memory,
        -1,
        1e-3,
        &source_memory);
    REQUIRE(V_memory == V_file);
    REQUIRE(F_memory == F_file);
    REQUIRE(source_memory == source_file);
}
TEST_CASE("MSH source IDs survive arbitrary node tags", "[mesh_data][io]")
{
    MeshData mesh;
    mesh.V.resize(3, 3);
    mesh.V << 0, 0, 0, 1, 0, 0, 0, 1, 0;
    mesh.cells[CellType::Triangle].connectivity.resize(1, 3);
    mesh.cells[CellType::Triangle].connectivity << 0, 1, 2;
    auto spec = io::to_msh(mesh);
    const std::vector<size_t> ids = {42, 99, 7};
    spec.nodes.entity_blocks[0].tags = ids;
    for (auto& b : spec.elements.entity_blocks)
        for (size_t i = 0; i < b.num_elements_in_block; ++i)
            for (int j = 1; j <= 3; ++j) b.data[i * 4 + j] = ids.at(b.data[i * 4 + j] - 1);
    const auto loaded = io::from_msh(spec);
    REQUIRE(
        loaded.cell(CellType::Triangle).connectivity == mesh.cell(CellType::Triangle).connectivity);
    REQUIRE(
        loaded.vertex_attributes.at("vid").integers() ==
        (DataMatrix<int64_t>(3, 1) << 41, 98, 6).finished());
}
TEST_CASE("A reused memory writer starts each mesh without fields", "[mesh_data][io]")
{
    MeshMap outputs;
    io::DataWriter writer(&outputs);
    Eigen::MatrixXd V(3, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    writer.add_field("quality", Eigen::MatrixXd::Ones(3, 1));
    writer.add_cell_field("energy", Eigen::MatrixXd::Ones(1, 1));
    writer.write_mesh("first.vtu", V, F, paraviewo::CellType::Triangle);
    Eigen::MatrixXd E_V(2, 3);
    E_V << 0, 0, 0, 1, 0, 0;
    Eigen::MatrixXi E(1, 2);
    E << 0, 1;
    REQUIRE_NOTHROW(writer.write_mesh("second.vtu", E_V, E, paraviewo::CellType::Line));
    REQUIRE(outputs.at("first.vtu").vertex_attributes.count("quality"));
    REQUIRE(outputs.at("second.vtu").vertex_attributes.empty());
    REQUIRE(outputs.at("second.vtu").cell(CellType::Edge).attributes.empty());
}
TEST_CASE("Memory exports map to result keys explicitly", "[mesh_data][io]")
{
    auto raw = [] {
        MeshMap m;
        m["mesh.vtu"] = MeshData();
        m["mesh_surf.vtu"] = MeshData();
        return m;
    };
    const auto named = io::named_outputs(raw());
    REQUIRE(named.size() == 2);
    REQUIRE(named.count("mesh"));
    REQUIRE(named.count("surface"));
    auto keys = io::default_output_keys();
    keys["mesh_surf.vtu"] = "";
    REQUIRE(io::named_outputs(raw(), keys).size() == 1);
    keys["mesh_surf.vtu"] = "mesh";
    REQUIRE_THROWS(io::named_outputs(raw(), keys));
    auto unknown = raw();
    unknown["mesh_offset.vtu"] = MeshData();
    REQUIRE_THROWS(io::named_outputs(std::move(unknown)));
}
TEST_CASE("MSH physical groups keep their file order", "[mesh_data][io]")
{
    MeshData mesh;
    mesh.V.resize(4, 3);
    mesh.V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    mesh.cells[CellType::Tetrahedron].connectivity.resize(1, 4);
    mesh.cells[CellType::Tetrahedron].connectivity << 0, 1, 2, 3;
    const std::vector<std::string> labels = {"ambient", "tag_2", "tag_10"};
    mesh.cells[CellType::Tetrahedron].attributes["tags"] = {
        DataMatrix<uint8_t>::Ones(1, 3).eval(),
        labels};
    auto spec = io::to_msh(mesh);
    // Keep only the physical groups, as gmsh and the simwild writer store region membership.
    spec.element_data.clear();
    const auto loaded = io::from_msh(spec);
    const auto& tags = loaded.cell(CellType::Tetrahedron).attributes.at("tags");
    REQUIRE(tags.labels == labels);
    REQUIRE(tags.integers() == DataMatrix<int64_t>::Ones(1, 3));
}
TEST_CASE("Undefined floating diagnostics round trip", "[mesh_data][io]")
{
    MeshData mesh;
    mesh.V = DataMatrix<double>::Zero(3, 3);
    DataMatrix<double> field(3, 1);
    field << std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity();
    mesh.vertex_attributes["diagnostic"].values = field;
    TempMesh file(".msh");
    io::write_mesh(file.path, mesh);
    const auto result = io::read_mesh(file.path).vertex_attributes.at("diagnostic").doubles();
    REQUIRE(std::isnan(result(0, 0)));
    REQUIRE(result(1, 0) == field(1, 0));
    REQUIRE(result(2, 0) == field(2, 0));
}
TEST_CASE("Mesh I/O retains coincident cell multiplicity and orientation", "[mesh_data][io]")
{
    MeshData mesh;
    mesh.V.resize(3, 3);
    mesh.V << 0, 0, 0, 1, 0, 0, 0, 1, 0;
    auto& block = mesh.cells[CellType::Triangle];
    block.connectivity.resize(2, 3);
    block.connectivity << 0, 1, 2, 0, 2, 1;
    block.attributes["value"].values = (DataMatrix<int64_t>(2, 1) << 7, 9).finished();
    TempMesh file(".msh");
    io::write_mesh(file.path, mesh);
    equal(mesh, io::read_mesh(file.path));
}
TEST_CASE("MeshData VTU output uses the existing writer", "[mesh_data][io]")
{
    auto mesh = example();
    mesh.vertex_attributes.clear();
    for (auto& [type, block] : mesh.cells) block.attributes.clear();
    const std::vector<paraviewo::CellType> types = {
        paraviewo::CellType::Vertex,
        paraviewo::CellType::Line,
        paraviewo::CellType::Triangle,
        paraviewo::CellType::Quadrilateral,
        paraviewo::CellType::Tetrahedron,
        paraviewo::CellType::Wedge,
        paraviewo::CellType::Pyramid};
    std::vector<paraviewo::CellElement> cells;
    size_t t = 0;
    for (const auto& [type, block] : mesh.cells) {
        for (Eigen::Index i = 0; i < block.connectivity.rows(); ++i) {
            paraviewo::CellElement cell;
            cell.ctype = types.at(t);
            for (Eigen::Index j = 0; j < block.connectivity.cols(); ++j)
                cell.vertices.push_back(int(block.connectivity(i, j)));
            cells.push_back(std::move(cell));
        }
        ++t;
    }
    paraviewo::VTUWriter legacy;
    SECTION("mixed geometry") {}
    SECTION("point and mixed cell attributes")
    {
        DataMatrix<int64_t> ids = DataMatrix<int64_t>::Constant(mesh.V.rows(), 1, 42);
        mesh.vertex_attributes["vid"].values = ids;
        mesh.vertex_attributes["position"].values = mesh.V;
        legacy.add_field("position", mesh.V);
        legacy.add_field("vid", ids.cast<double>());
        Eigen::MatrixXd quality(cells.size(), 1);
        Eigen::Index row = 0;
        for (auto& [type, block] : mesh.cells) {
            auto values =
                DataMatrix<double>::Constant(block.connectivity.rows(), 1, double(int(type)))
                    .eval();
            block.attributes["quality"].values = values;
            quality.middleRows(row, values.rows()) = values;
            row += values.rows();
        }
        legacy.add_cell_field("quality", quality);
    }
    TempMesh file(".vtu"), reference(".vtu");
    io::write_mesh(file.path, mesh);
    REQUIRE(legacy.write_mesh(reference.path.string(), mesh.V, cells));
    std::ifstream actual(file.path, std::ios::binary), expected(reference.path, std::ios::binary);
    REQUIRE(
        std::string(std::istreambuf_iterator<char>(actual), {}) ==
        std::string(std::istreambuf_iterator<char>(expected), {}));
}
TEST_CASE("MeshData VTU output rounds tiny values like the component writers", "[mesh_data][io]")
{
    MeshData mesh;
    mesh.V = DataMatrix<double>::Zero(3, 3);
    mesh.cells[CellType::Triangle].connectivity = (DataMatrix<int64_t>(1, 3) << 0, 1, 2).finished();
    mesh.vertex_attributes["diagnostic"].values = DataMatrix<double>::Constant(3, 1, 1e-20).eval();
    TempMesh file(".vtu");
    REQUIRE_NOTHROW(io::write_mesh(file.path, mesh));
    REQUIRE(std::filesystem::exists(file.path));
}
TEST_CASE("MeshData VTU adapter rejects unsupported data", "[mesh_data][io]")
{
    MeshData mesh;
    mesh.V = DataMatrix<double>::Zero(3, 3);
    auto& block = mesh.cells[CellType::Triangle];
    block.connectivity = (DataMatrix<int64_t>(1, 3) << 0, 1, 2).finished();
    SECTION("attribute labels")
    {
        mesh.vertex_attributes["tag"] = {DataMatrix<double>::Ones(3, 1).eval(), {"group"}};
    }
    SECTION("two-column attributes")
    {
        mesh.vertex_attributes["uv"].values = DataMatrix<double>::Ones(3, 2).eval();
    }
    SECTION("integer precision")
    {
        mesh.vertex_attributes["vid"].values =
            DataMatrix<int64_t>::Constant(3, 1, 9007199254740993LL).eval();
    }
    SECTION("attribute names requiring XML escaping")
    {
        mesh.vertex_attributes["a&b"].values = DataMatrix<double>::Ones(3, 1).eval();
    }
    SECTION("attributes missing from another cell block")
    {
        block.attributes["quality"].values = DataMatrix<double>::Ones(1, 1).eval();
        mesh.cells[CellType::Edge].connectivity = (DataMatrix<int64_t>(1, 2) << 0, 1).finished();
    }
    TempMesh file(".vtu");
    REQUIRE_THROWS(io::write_mesh(file.path, mesh));
    REQUIRE_FALSE(std::filesystem::exists(file.path));
}
