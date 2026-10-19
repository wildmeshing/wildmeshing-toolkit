#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/components/prismatic_mesh/prismatic_mesh.hpp>

using namespace wmtk::components::prismatic_mesh;

namespace {
using Tet = std::array<size_t, 4>;

PrismaticMeshInput fixture(std::vector<Tet> tets)
{
    PrismaticMeshInput data;
    data.vertices.resize(12, 3);
    data.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, -0.2, -0.2, 1, -0.2, -0.2, -1, -0.4, -0.3, 0.5, -1,
        0.2, 0, 0.1, -1, 0, 0.1, 0.1, -2, 0.3, 0.4, 2, 2, 1, 0.5, 1, -2, 1;
    data.vertex_tags.assign(12, 1);
    data.source_vertex_ids.resize(12);
    data.corr_input_vid.assign(12, -1);
    data.corr_input_vertex.assign(12, -1);
    data.input_to_offset_vertices.resize(12);
    for (size_t v = 0; v < 12; ++v) data.source_vertex_ids[v] = 101 + 17 * v;
    for (size_t v : {3, 4, 5, 9}) {
        data.vertex_tags[v] = 2;
        data.offset_vertices.push_back(v);
        data.corr_input_vertex[v] = 0;
        data.corr_input_vid[v] = data.source_vertex_ids[0];
        data.input_to_offset_vertices[0].push_back(v);
    }
    for (size_t v = 0; v < 12; ++v)
        if (data.vertex_tags[v] == 1) data.input_vertices.push_back(v);
    data.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        if (!tet_volume_above_threshold(data.vertices, tets[i], 0))
            std::swap(tets[i][0], tets[i][1]);
        REQUIRE(tet_volume_above_threshold(data.vertices, tets[i], 0));
        for (int j = 0; j < 4; ++j) data.tetrahedra(i, j) = tets[i][j];
    }
    data.mesh = std::make_unique<wmtk::TetMesh>();
    data.mesh->init_with_isolated_vertices(data.vertices.rows(), tets);
    data.input_cells.assign(tets.size(), 0);
    data.offset_tet_tags.assign(tets.size(), 1);
    return data;
}

void parent(PrismaticMeshInput& data, size_t offset, size_t input)
{
    auto& old = data.input_to_offset_vertices[data.corr_input_vertex[offset]];
    old.erase(std::remove(old.begin(), old.end(), offset), old.end());
    data.corr_input_vertex[offset] = static_cast<int64_t>(input);
    data.corr_input_vid[offset] = data.source_vertex_ids[input];
    data.input_to_offset_vertices[input].push_back(offset);
}
} // namespace

TEST_CASE("open boundary applies all six shell types and the matching tau22 exception")
{
    Tet candidate{0, 1, 2, 4};
    bool unmark = false;
    size_t parent4 = 0, parent5 = 0;
    SECTION("tau11 is preserved on both sides of one unique input face") {}
    SECTION("tau21 is unmarked")
    {
        candidate = {0, 1, 3, 4};
        unmark = true;
    }
    SECTION("tau22 matching the boundary input edge is unmarked")
    {
        candidate = {0, 1, 3, 4};
        parent4 = 1;
        unmark = true;
    }
    SECTION("tau22 with different input and offset identifier sets is preserved")
    {
        candidate = {0, 1, 3, 4};
        parent4 = 2;
    }
    SECTION("tau31 in an endpoint star is unmarked")
    {
        candidate = {0, 3, 4, 5};
        unmark = true;
    }
    SECTION("tau32 mapping to a boundary edge is unmarked")
    {
        candidate = {0, 3, 4, 5};
        parent4 = parent5 = 1;
        unmark = true;
    }
    SECTION("tau33 is preserved")
    {
        candidate = {0, 3, 4, 5};
        parent4 = 1;
        parent5 = 2;
    }
    auto data = fixture({{0, 1, 2, 3}, candidate});
    parent(data, 4, parent4);
    parent(data, 5, parent5);
    const auto* mesh = data.mesh.get();
    const auto vertices = data.vertices;
    const auto tets = data.tetrahedra;
    const auto tags = data.vertex_tags;
    const auto ids = data.source_vertex_ids;
    const auto corr = data.corr_input_vid;
    const auto corr_rows = data.corr_input_vertex;
    const auto reverse = data.input_to_offset_vertices;
    const auto input_cells = data.input_cells;
    const auto stats = handle_open_boundaries(data);
    REQUIRE(stats.open_boundary_edges == 3);
    REQUIRE(stats.candidate_tets == 2);
    REQUIRE(stats.unmarked_tets == (unmark ? 1 : 0));
    REQUIRE(data.offset_tet_tags == std::vector<int>{1, unmark ? -1 : 1});
    REQUIRE(data.mesh.get() == mesh);
    REQUIRE(data.mesh->get_tets().size() == 2);
    REQUIRE(data.vertices == vertices);
    REQUIRE(data.tetrahedra == tets);
    REQUIRE(data.vertex_tags == tags);
    REQUIRE(data.source_vertex_ids == ids);
    REQUIRE(data.corr_input_vid == corr);
    REQUIRE(data.corr_input_vertex == corr_rows);
    REQUIRE(data.input_to_offset_vertices == reverse);
    REQUIRE(data.input_cells == input_cells);
    REQUIRE(handle_open_boundaries(data).unmarked_tets == 0);
}

TEST_CASE("tau32 mapping to an interior input edge is preserved near an open boundary")
{
    auto data = fixture({{0, 1, 2, 3}, {0, 2, 6, 3}, {0, 3, 4, 5}});
    parent(data, 4, 2);
    parent(data, 5, 2);
    const auto stats = handle_open_boundaries(data);
    REQUIRE(stats.open_boundary_edges == 4);
    REQUIRE(stats.candidate_tets == 3);
    REQUIRE(stats.unmarked_tets == 0);
    REQUIRE(data.offset_tet_tags == std::vector<int>{1, 1, 1});
}

TEST_CASE("open boundary excludes volume edges using full mesh cell membership")
{
    auto data = fixture({{0, 1, 2, 3}, {0, 1, 3, 4}, {0, 2, 1, 8}});
    data.input_cells[2] = 1;
    data.offset_tet_tags[2] = -1;
    const auto stats = handle_open_boundaries(data);
    REQUIRE(stats.open_boundary_edges == 0);
    REQUIRE(stats.candidate_tets == 0);
    REQUIRE(data.offset_tet_tags == std::vector<int>{1, 1, -1});
}

TEST_CASE("a volume vertex alone does not make an incident shell edge volumetric")
{
    auto data = fixture({{0, 1, 2, 3}, {0, 1, 3, 4}, {0, 6, 7, 8}});
    data.input_cells[2] = 1;
    data.offset_tet_tags[2] = -1;
    const auto stats = handle_open_boundaries(data);
    REQUIRE(stats.open_boundary_edges == 3);
    REQUIRE(stats.unmarked_tets == 1);
    REQUIRE(data.offset_tet_tags == std::vector<int>{1, -1, -1});
    REQUIRE(data.input_cells == std::vector<int>{0, 0, 1});
}

TEST_CASE("only shell input faces count and only input boundary endpoint stars are candidates")
{
    auto data = fixture({{0, 1, 2, 3}, {0, 3, 4, 5}, {6, 3, 4, 5}, {0, 1, 6, 10}});
    data.offset_tet_tags[3] = -1; // All-input background face is not a shell face.
    const auto stats = handle_open_boundaries(data);
    REQUIRE(stats.open_boundary_edges == 3);
    REQUIRE(stats.candidate_tets == 2);
    REQUIRE(stats.unmarked_tets == 1);
    REQUIRE(data.offset_tet_tags == std::vector<int>{1, -1, 1, -1});
}

TEST_CASE("closed shell input surfaces and empty bands have no open boundary")
{
    auto data = fixture({{0, 1, 2, 3}, {0, 2, 6, 4}, {0, 6, 1, 5}, {1, 6, 2, 9}});
    SECTION("closed input surface") {}
    SECTION("empty band")
    {
        data.offset_tet_tags.assign(4, -1);
    }
    const auto tags = data.offset_tet_tags;
    const auto stats = handle_open_boundaries(data);
    REQUIRE(stats.open_boundary_edges == 0);
    REQUIRE(stats.candidate_tets == 0);
    REQUIRE(data.offset_tet_tags == tags);
}

TEST_CASE("open boundary separates sheet target components and retains unmarked background cells")
{
    auto data = fixture({{0, 1, 2, 3}, {0, 2, 1, 4}, {0, 1, 3, 4}, {0, 2, 4, 3}});
    const auto vertices = data.vertices;
    const auto tets = data.tetrahedra;
    OptimizationOptions options;
    options.iterations = 0;
    options.keep_background_mesh = true;
    SECTION("with background") {}
    SECTION("band only")
    {
        options.keep_background_mesh = false;
    }
    prism_main(data, 0.1, options);
    REQUIRE(data.vertices == vertices);
    REQUIRE(data.tetrahedra == tets.topRows(options.keep_background_mesh ? 4 : 2));
    REQUIRE(data.offset_components.size() == 2);
    REQUIRE(data.vertex_component_ids[3] != data.vertex_component_ids[4]);
    REQUIRE(data.singular_vertex_tags[3] == 0);
    REQUIRE(data.singular_vertex_tags[4] == 0);
    REQUIRE((data.optimal_normals.row(3) - wmtk::Vector3d::UnitZ().transpose()).norm() < 1e-12);
    REQUIRE((data.optimal_normals.row(4) + wmtk::Vector3d::UnitZ().transpose()).norm() < 1e-12);
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
    REQUIRE(std::count(data.offset_tet_tags.begin(), data.offset_tet_tags.end(), 1) == 2);
    for (const auto& tet : data.mesh->get_tets())
        REQUIRE(tet_volume_above_threshold(data.vertices, data.mesh->oriented_tet_vids(tet), 0));
    if (options.keep_background_mesh)
        REQUIRE(data.offset_tet_tags == std::vector<int>{1, 1, -1, -1});
}

TEST_CASE("T junction vertices protect their complete one-rings before open boundary trimming")
{
    // Three two-sided input triangles meet along {0,1}. The last three tets are
    // tau31s: two touch just one junction endpoint; the last only touches input 2.
    auto data = fixture(
        {{0, 1, 2, 3},
         {0, 1, 3, 7},
         {0, 1, 7, 4},
         {0, 1, 4, 8},
         {0, 1, 8, 5},
         {0, 1, 5, 2},
         {0, 3, 4, 5},
         {1, 3, 4, 9},
         {2, 3, 4, 9}});
    const auto tets = data.tetrahedra;
    const auto stats = handle_open_boundaries(data);
    REQUIRE(stats.open_boundary_edges == 6);
    REQUIRE(stats.junction_vertices == 2);
    REQUIRE(stats.candidate_tets == 9);
    REQUIRE(stats.protected_candidate_tets == 8);
    REQUIRE(stats.unmarked_tets == 1);
    REQUIRE(data.offset_tet_tags == std::vector<int>{1, 1, 1, 1, 1, 1, 1, 1, -1});
    REQUIRE(data.tetrahedra == tets);
    for (size_t junction : {0, 1})
        for (size_t tid : data.mesh->get_one_ring_tids_for_vertex(junction))
            REQUIRE(data.offset_tet_tags[tid] == 1);
}
