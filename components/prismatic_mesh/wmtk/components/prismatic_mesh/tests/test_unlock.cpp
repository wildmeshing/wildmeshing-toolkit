#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/components/prismatic_mesh/prismatic_mesh.hpp>

using namespace wmtk::components::prismatic_mesh;

namespace {
PrismaticMeshInput fixture(bool single = false)
{
    PrismaticMeshInput data;
    data.vertices.resize(6, 3);
    data.vertices << 0, 0, -1, 1, 0, 0, 0, 1, 0, 0, 0, 1, -1, 0, 0, 0, -1, 0;
    std::vector<std::array<size_t, 4>> tets = {
        {0, 3, 1, 2},
        {0, 3, 2, 4},
        {0, 3, 4, 5},
        {0, 3, 5, 1}};
    if (single) tets.resize(1);
    data.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        if (!tet_volume_above_threshold(data.vertices, tets[i], 0))
            std::swap(tets[i][0], tets[i][1]);
        REQUIRE(tet_volume_above_threshold(data.vertices, tets[i], 0));
        for (int j = 0; j < 4; ++j) data.tetrahedra(i, j) = tets[i][j];
    }
    data.mesh = std::make_unique<wmtk::TetMesh>();
    data.mesh->init_with_isolated_vertices(6, tets);
    data.vertex_tags = {1, 1, 2, 2, -1, -1};
    data.source_vertex_ids = {100, 500, 200, 300, 400, 600};
    data.corr_input_vid = {-1, -1, 100, 500, -1, -1};
    data.corr_input_vertex = {-1, -1, 0, 1, -1, -1};
    data.vertex_component_ids = {-1, -1, 0, 1, -1, -1};
    data.singular_vertex_tags = {-1, -1, 0, 0, -1, -1};
    data.input_vertices = {0, 1};
    data.offset_vertices = {2, 3};
    data.input_to_offset_vertices.resize(6);
    data.input_to_components.resize(6);
    data.optimal_normals = wmtk::MatrixXd::Zero(6, 3);
    data.target_positions = data.vertices;
    for (size_t i = 0; i < 2; ++i) {
        OffsetComponent component;
        component.input_vertex = i;
        component.vertices = {i + 2};
        component.target_position = data.vertices.row(i + 2);
        component.optimal_normal = wmtk::Vector3d::UnitY();
        component.singular = false;
        data.offset_components.push_back(component);
        data.input_to_offset_vertices[i] = {i + 2};
        data.input_to_components[i] = {i};
        data.optimal_normals.row(i + 2) = component.optimal_normal.transpose();
    }
    data.input_cells.assign(tets.size(), 0);
    data.offset_tet_tags.assign(tets.size(), 1);
    return data;
}

void check_mesh(const PrismaticMeshInput& data, double floor)
{
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
    REQUIRE(data.tetrahedra.rows() == data.mesh->get_tets().size());
    REQUIRE(data.input_cells.size() == data.mesh->get_tets().size());
    REQUIRE(data.offset_tet_tags.size() == data.mesh->get_tets().size());
    for (const auto& t : data.mesh->get_tets()) {
        const auto ids = data.mesh->oriented_tet_vids(t);
        REQUIRE(tet_volume_above_threshold(data.vertices, ids, floor));
        REQUIRE(data.offset_tet_tags[t.tid(*data.mesh)] == 1);
        for (int j = 0; j < 4; ++j) REQUIRE(data.tetrahedra(t.tid(*data.mesh), j) == ids[j]);
    }
}
} // namespace

TEST_CASE("tau22 classification matches original IDs and requires two distinct correspondences")
{
    auto data = fixture();
    auto tau = classify_tau22(data, 0);
    REQUIRE(tau);
    REQUIRE(tau->input_vertices == std::array<size_t, 2>{0, 1});
    REQUIRE(tau->offset_vertices == std::array<size_t, 2>{2, 3});
    REQUIRE_FALSE(classify_tau22(data, 1));
    REQUIRE_FALSE(classify_tau22(data, 100));
    SECTION("equal correspondences")
    {
        data.corr_input_vid[3] = 100;
    }
    SECTION("unmatched correspondence")
    {
        data.corr_input_vid[3] = 600;
    }
    SECTION("other vertex")
    {
        data.vertex_tags[3] = -1;
    }
    SECTION("three input vertices")
    {
        data.vertex_tags[3] = 1;
    }
    REQUIRE_FALSE(classify_tau22(data, 0));
}

TEST_CASE("unlock uses each symmetric cross edge and preserves every original vertex attribute")
{
    int side = 0;
    SECTION("input a offset b then x to offset a") {}
    SECTION("input b offset a then x to offset b")
    {
        side = 1;
    }
    auto data = fixture();
    auto before = fixture();
    REQUIRE(try_unlock_tau22(data, 0, side, 1e-6));
    REQUIRE(data.vertices == before.vertices);
    REQUIRE(data.mesh->vert_capacity() == before.mesh->vert_capacity());
    REQUIRE(data.mesh->get_vertices().size() == before.mesh->get_vertices().size());
    REQUIRE(data.vertex_tags == before.vertex_tags);
    REQUIRE(data.source_vertex_ids == before.source_vertex_ids);
    REQUIRE(data.corr_input_vid == before.corr_input_vid);
    REQUIRE(data.corr_input_vertex == before.corr_input_vertex);
    REQUIRE(data.vertex_component_ids == before.vertex_component_ids);
    REQUIRE(data.singular_vertex_tags == before.singular_vertex_tags);
    REQUIRE(data.optimal_normals == before.optimal_normals);
    REQUIRE(data.target_positions == before.target_positions);
    REQUIRE(data.offset_vertices == before.offset_vertices);
    REQUIRE(data.input_to_offset_vertices == before.input_to_offset_vertices);
    REQUIRE(data.input_to_components == before.input_to_components);
    for (size_t i = 0; i < data.offset_components.size(); ++i) {
        REQUIRE(data.offset_components[i].vertices == before.offset_components[i].vertices);
        REQUIRE(
            data.offset_components[i].optimal_normal == before.offset_components[i].optimal_normal);
        REQUIRE(
            data.offset_components[i].target_position ==
            before.offset_components[i].target_position);
    }
    for (const auto& t : data.mesh->get_tets())
        REQUIRE_FALSE(classify_tau22(data, t.tid(*data.mesh)));
    check_mesh(data, 1e-6);
}

TEST_CASE("rejected unlock rolls back the split including topology and all metadata")
{
    bool single = false;
    double floor = 0.2; // Original volume 1/3 passes; split children 1/6 fail.
    SECTION("split children below volume floor") {}
    SECTION("collapse cannot erase the whole component")
    {
        single = true;
        floor = 0;
    }
    auto data = fixture(single);
    auto before = fixture(single);
    auto* mesh = data.mesh.get();
    data.offset_face_tags = {123}; // Sentinel to verify even cached fields survive failure.
    for (int side = 0; side < 2; ++side) {
        REQUIRE_FALSE(try_unlock_tau22(data, 0, side, floor));
        REQUIRE(data.mesh.get() == mesh);
        REQUIRE(data.vertices == before.vertices);
        REQUIRE(data.tetrahedra == before.tetrahedra);
        REQUIRE(data.vertex_tags == before.vertex_tags);
        REQUIRE(data.offset_face_tags == std::vector<int>{123});
        REQUIRE(data.input_to_offset_vertices == before.input_to_offset_vertices);
        REQUIRE(data.source_vertex_ids == before.source_vertex_ids);
        REQUIRE(data.vertex_component_ids == before.vertex_component_ids);
    }
    check_mesh(data, floor);
}

TEST_CASE("optimization runs unlock before smoothing and records remaining tau22")
{
    auto data = fixture();
    data.offset_components[0].target_position = wmtk::Vector3d(0, 0.9, 0);
    data.target_positions.row(2) = data.offset_components[0].target_position.transpose();
    OptimizationOptions options;
    options.iterations = 2;
    options.min_tet_volume = 1e-6;
    optimize_prismatic_mesh(data, options);
    REQUIRE(data.optimization_iterations.size() == 2);
    const auto& first = data.optimization_iterations[0];
    REQUIRE(first.collapses == 0);
    REQUIRE(first.smoothed_vertices == 1);
    REQUIRE(first.unlock.candidates == 1);
    REQUIRE(first.unlock.attempted == 1);
    REQUIRE(first.unlock.unlocked == 1);
    REQUIRE(first.unlock.remaining == 0);
    REQUIRE(data.optimization_iterations[1].unlock.candidates == 0);
    REQUIRE(data.vertices.row(2).transpose() == data.offset_components[0].target_position);
    check_mesh(data, options.min_tet_volume);
    auto failed = fixture(true);
    auto stats = unlock_tau22(failed, 0);
    REQUIRE(stats.candidates == 1);
    REQUIRE(stats.attempted == 1);
    REQUIRE(stats.unlocked == 0);
    REQUIRE(stats.remaining == 1);
}

TEST_CASE("unlock rejects an inverting collapse and safely falls back to the symmetric direction")
{
    auto data = fixture();
    // A concave edge-star cross-section: all original/split tets are positive, but
    // moving the midpoint to offset a would invert the tets across the 4--5 edge.
    data.vertices.row(4) << -0.1, 0.1, 0;
    data.vertices.row(5) << -1, -1, 0;
    check_mesh(data, 1e-6);
    const auto vertices = data.vertices;
    const auto tets = data.tetrahedra;
    REQUIRE_FALSE(try_unlock_tau22(data, 0, 0, 1e-6));
    REQUIRE(data.vertices == vertices);
    REQUIRE(data.tetrahedra == tets);
    const auto stats = unlock_tau22(data, 1e-6);
    REQUIRE(stats.unlocked == 1);
    REQUIRE(stats.remaining == 0);
    REQUIRE(data.vertices == vertices);
    check_mesh(data, 1e-6);
}
