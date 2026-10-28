#include <mshio/mshio.h>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <wmtk/components/prismatic_mesh/prismatic_mesh.hpp>

using namespace wmtk::components::prismatic_mesh;
using Catch::Matchers::ContainsSubstring;
namespace {
struct MshFile
{
    std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("prismatic_msh_test_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".msh");
    void write(const mshio::MshSpec& spec) const
    {
        std::ofstream out(path, std::ios::binary);
        out << std::setprecision(17);
        mshio::save_msh(out, spec);
    }
    ~MshFile()
    {
        std::error_code ec;
        std::filesystem::remove(path, ec);
        std::filesystem::remove(
            path.parent_path() / (path.stem().string() + "_offset_faces.vtu"),
            ec);
    }
};
mshio::Data
field(const std::string& name, const std::vector<size_t>& tags, const std::vector<double>& values)
{
    mshio::Data data;
    data.header.string_tags = {name};
    data.header.real_tags = {0};
    data.header.int_tags = {0, 1, static_cast<int>(tags.size())};
    // Field order intentionally differs from the order of geometry records.
    for (size_t i = tags.size(); i-- > 0;) data.entries.push_back({tags[i], 0, {values[i]}});
    return data;
}
mshio::MshSpec construction(bool binary)
{
    mshio::MshSpec spec;
    spec.mesh_format.file_type = binary ? 1 : 0;
    spec.physical_groups = {{3, 17, "tag_0"}, {3, 29, "offset"}, {3, 53, "ambient"}};
    for (int i = 0; i < 3; ++i) {
        mshio::VolumeEntity entity;
        entity.tag = 10 + i;
        entity.physical_group_tags = {spec.physical_groups[i].tag};
        spec.entities.volumes.push_back(entity);
    }
    mshio::NodeBlock a, b;
    a.entity_dim = 3;
    a.entity_tag = 10;
    a.num_nodes_in_block = 3;
    a.tags = {41, 7, 100};
    a.data = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    b.entity_dim = 3;
    b.entity_tag = 11;
    b.num_nodes_in_block = 3;
    b.tags = {9, 55, 88};
    b.data = {0, 0, 1, 0, 0, -0.1, 0, 0, -2};
    spec.nodes.entity_blocks = {a, b};
    spec.nodes.num_entity_blocks = 2;
    spec.nodes.num_nodes = 6;
    spec.nodes.min_node_tag = 7;
    spec.nodes.max_node_tag = 100;
    const std::array<std::array<size_t, 5>, 3> cells = {
        {{901, 41, 7, 100, 9}, {42, 41, 100, 7, 55}, {1234, 100, 7, 55, 88}}};
    for (int i = 0; i < 3; ++i) {
        mshio::ElementBlock block;
        block.entity_dim = 3;
        block.entity_tag = 10 + i;
        block.element_type = 4;
        block.num_elements_in_block = 1;
        block.data.assign(cells[i].begin(), cells[i].end());
        spec.elements.entity_blocks.push_back(block);
    }
    spec.elements.num_entity_blocks = 3;
    spec.elements.num_elements = 3;
    spec.elements.min_element_tag = 42;
    spec.elements.max_element_tag = 1234;
    spec.node_data = {field("corr_input_vid", {41, 7, 100, 9, 55, 88}, {-1, -1, -1, -1, 40, -1})};
    return spec;
}
} // namespace

TEST_CASE(
    "MSH construction imports ASCII and binary with noncontiguous tags and shuffled fields",
    "[prismatic_msh]")
{
    const bool binary = GENERATE(false, true);
    auto spec = construction(binary);
    // Empty blocks occur in construction exports and must not cause division by zero.
    auto empty = spec.elements.entity_blocks.back();
    empty.num_elements_in_block = 0;
    empty.data.clear();
    spec.elements.entity_blocks.push_back(empty);
    ++spec.elements.num_entity_blocks;
    MshFile file;
    file.write(spec);
    const auto mesh = load_prismatic_mesh(file.path);
    REQUIRE(mesh.vertices.rows() == 6);
    REQUIRE(mesh.vertices(4, 2) == -0.1);
    REQUIRE(mesh.tetrahedra.rows() == 3);
    REQUIRE(mesh.tetrahedra(1, 1) == 2);
    REQUIRE(mesh.tetrahedra(2, 3) == 5);
    REQUIRE(mesh.source_vertex_ids == std::vector<int64_t>{40, 6, 99, 8, 54, 87});
    REQUIRE(mesh.vertex_tags == std::vector<int>{1, 1, 1, 1, 2, -1});
    REQUIRE(mesh.corr_input_vertex == std::vector<int64_t>{-1, -1, -1, -1, 0, -1});
    REQUIRE(mesh.input_to_offset_vertices[0] == std::vector<size_t>{4});
    REQUIRE(mesh.input_cells == std::vector<int>{1, 0, 0});
    REQUIRE(mesh.offset_tet_tags == std::vector<int>{-1, 1, -1});
}

TEST_CASE(
    "MSH explicit attributes preserve arbitrary source IDs and roundtrip tetrahedral exports",
    "[prismatic_msh]")
{
    auto spec = construction(GENERATE(false, true));
    const std::vector<size_t> tags = {41, 7, 100, 9, 55, 88};
    spec.node_data = {
        field("vid", tags, {500, 600, 700, 800, 900, 1000}),
        field("labels", tags, {1, 1, 1, 1, 2, 0}),
        field("corr_input_vid", tags, {-1, -1, -1, -1, 500, -1})};
    spec.element_data = {
        field("tag_0", {901, 42, 1234}, {1, 0, 0}),
        field("offset_tag", {901, 42, 1234}, {0, 1, 0})};
    spec.physical_groups.clear(); // Explicit fields are sufficient, without PhysicalNames.
    MshFile source;
    source.write(spec);
    auto mesh = load_prismatic_mesh(source.path);
    REQUIRE(mesh.source_vertex_ids == std::vector<int64_t>{500, 600, 700, 800, 900, 1000});
    REQUIRE(mesh.corr_input_vertex[4] == 0);
    REQUIRE(mesh.input_cells == std::vector<int>{1, 0, 0});
    REQUIRE(mesh.offset_tet_tags == std::vector<int>{-1, 1, -1});
    label_offset_faces(mesh);
    MshFile output;
    write_prismatic_mesh(mesh, output.path);
    const auto reread = load_prismatic_mesh(output.path);
    std::map<int64_t, size_t> rows;
    for (size_t i = 0; i < reread.source_vertex_ids.size(); ++i)
        rows[reread.source_vertex_ids[i]] = i;
    for (size_t i = 0; i < mesh.source_vertex_ids.size(); ++i) {
        const auto row = rows.at(mesh.source_vertex_ids[i]);
        REQUIRE(reread.vertices.row(row) == mesh.vertices.row(i));
        REQUIRE(reread.vertex_tags[row] == mesh.vertex_tags[i]);
        REQUIRE(reread.corr_input_vid[row] == mesh.corr_input_vid[i]);
    }
    REQUIRE(reread.mesh->get_tets().size() == 3);
}

TEST_CASE("MSH rejects ambiguous or invalid construction metadata", "[prismatic_msh]")
{
    auto spec = construction(false);
    std::string error;
    SECTION("missing correspondence")
    {
        spec.node_data.clear();
        error = "missing corr_input_vid";
    }
    SECTION("unknown correspondence")
    {
        spec.node_data[0].entries[1].data[0] = 12345;
        error = "unknown input correspondence";
    }
    SECTION("correspondence to background")
    {
        spec.node_data[0].entries[1].data[0] = 87;
        error = "not an input vertex";
    }
    SECTION("fractional correspondence")
    {
        spec.node_data[0].entries[1].data[0] = 40.5;
        error = "invalid corr_input_vid";
    }
    SECTION("incomplete field")
    {
        spec.node_data[0].entries.pop_back();
        --spec.node_data[0].header.int_tags[2];
        error = "cover every";
    }
    SECTION("truncated field payload")
    {
        spec.node_data[0].entries.pop_back();
        error = "malformed or truncated";
    }
    SECTION("duplicate data tag")
    {
        spec.node_data[0].entries[0].tag = spec.node_data[0].entries[1].tag;
        error = "duplicate node/element tag";
    }
    SECTION("unknown data tag")
    {
        spec.node_data[0].entries[0].tag = 999;
        error = "unknown node/element tag";
    }
    SECTION("duplicate field")
    {
        spec.node_data.push_back(spec.node_data.front());
        error = "duplicate field";
    }
    SECTION("duplicate node tag")
    {
        spec.nodes.entity_blocks[1].tags[0] = 41;
        error = "duplicate node tag";
    }
    SECTION("unknown connectivity")
    {
        spec.elements.entity_blocks[1].data[1] = 999;
        error = "unknown node tag";
    }
    SECTION("duplicate element tag")
    {
        spec.elements.entity_blocks[1].data[0] = 901;
        error = "duplicate element tag";
    }
    SECTION("duplicate tetrahedron")
    {
        auto& b = spec.elements.entity_blocks[1];
        b.data = spec.elements.entity_blocks[0].data;
        b.data[0] = 42;
        error = "duplicate tetrahedron";
    }
    SECTION("repeated corner")
    {
        spec.elements.entity_blocks[0].data[2] = 41;
        error = "repeated vertices";
    }
    SECTION("unknown region")
    {
        spec.physical_groups[1].name = "unknown";
        error = "missing input/band/background";
    }
    SECTION("conflicting regions")
    {
        spec.entities.volumes[0].physical_group_tags.push_back(29);
        error = "conflicting volume physical groups";
    }
    SECTION("partial cell attributes")
    {
        spec.element_data.push_back(field("tag_0", {901, 42, 1234}, {1, 0, 0}));
        error = "both be present";
    }
    SECTION("hybrid wedge")
    {
        auto& b = spec.elements.entity_blocks[1];
        b.element_type = 6;
        b.data = {42, 41, 7, 100, 9, 55, 88};
        error = "only four-node tetrahedra";
    }
    MshFile file;
    file.write(spec);
    REQUIRE_THROWS_WITH(load_prismatic_mesh(file.path), ContainsSubstring(error));
}
