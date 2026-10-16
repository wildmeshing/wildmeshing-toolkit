#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <wmtk/components/prismatic_mesh/prismatic_mesh.hpp>

using namespace wmtk::components::prismatic_mesh;
using wmtk::Vector3d;

namespace {
PrismaticMeshInput sheet(bool extra_components = false)
{
    PrismaticMeshInput data;
    data.vertices.resize(8, 3);
    data.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -1, 0.2, 0.2, 1, 0.7, 0.3, 1, 0.8,
        0.4, 1.2;
    std::vector<std::array<size_t, 4>> tets = {{0, 1, 2, 3}, {0, 2, 1, 4}, {0, 1, 3, 5}};
    if (extra_components) {
        tets.push_back({0, 1, 5, 6});
        tets.push_back({0, 1, 6, 7});
    }
    data.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i)
        for (int j = 0; j < 4; ++j) data.tetrahedra(i, j) = tets[i][j];
    data.mesh = std::make_unique<wmtk::TetMesh>();
    data.mesh->init_with_isolated_vertices(8, tets);
    data.vertex_tags = {1, 1, 1, 2, 2, 2, 2, 2};
    data.source_vertex_ids = {100, 101, 102, 103, 104, 105, 106, 107};
    data.corr_input_vid = {-1, -1, -1, 100, 100, 100, 101, 100};
    data.corr_input_vertex = {-1, -1, -1, 0, 0, 0, 1, 0};
    data.input_vertices = {0, 1, 2};
    data.offset_vertices = {3, 4, 5, 6, 7};
    data.input_to_offset_vertices.resize(8);
    data.input_to_offset_vertices[0] = {3, 4, 5, 7};
    data.input_to_offset_vertices[1] = {6};
    data.input_cells.assign(tets.size(), 0);
    data.offset_tet_tags.assign(tets.size(), 1);
    return data;
}
} // namespace

TEST_CASE("LU target directions handle planes wedges corners and conflicting normals")
{
    Vector3d d;
    const Vector3d n = Vector3d(1, 2, 3).normalized();
    REQUIRE(solve_target_direction({n, n, n}, d));
    REQUIRE((d - n).norm() < 1e-10);
    REQUIRE(solve_target_direction({Vector3d::UnitX(), Vector3d::UnitY()}, d));
    REQUIRE((d - Vector3d(1, 1, 0).normalized()).norm() < 1e-10);
    REQUIRE(solve_target_direction({Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()}, d));
    REQUIRE((d - Vector3d(1, 1, 1).normalized()).norm() < 1e-10);
    REQUIRE_FALSE(solve_target_direction({n, -n}, d));
    REQUIRE(d.isZero());
    REQUIRE_FALSE(solve_target_direction({}, d));
    REQUIRE_FALSE(solve_target_direction({Vector3d::Zero()}, d));
    REQUIRE_FALSE(
        solve_target_direction({Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 0)}, d));
}

TEST_CASE(
    "a two-sided sheet gets two components with opposite normals and a common target per component")
{
    auto data = sheet();
    const wmtk::MatrixXd original_positions = data.vertices;
    evaluate_target_positions(data, 0.3);
    REQUIRE(data.vertices == original_positions); // evaluation does not move the mesh
    REQUIRE(data.offset_components.size() == 2);
    REQUIRE(data.input_to_components[0].size() == 2);
    REQUIRE(data.vertex_component_ids[3] == data.vertex_component_ids[5]);
    REQUIRE(data.vertex_component_ids[3] != data.vertex_component_ids[4]);
    REQUIRE(data.vertex_component_ids[0] == -1);
    REQUIRE(data.vertex_component_ids[6] == -1); // inactive offset vertex is excluded
    REQUIRE(data.singular_vertex_tags[6] == -1);
    REQUIRE(std::abs(data.input_average_edge_length - (2 + std::sqrt(2.0)) / 3) < 1e-12);
    REQUIRE(std::abs(data.target_thickness - 0.3 * data.input_average_edge_length) < 1e-12);
    const auto& upper = data.offset_components[data.vertex_component_ids[3]];
    const auto& lower = data.offset_components[data.vertex_component_ids[4]];
    REQUIRE_FALSE(upper.singular);
    REQUIRE_FALSE(lower.singular);
    REQUIRE((upper.optimal_normal - Vector3d::UnitZ()).norm() < 1e-12);
    REQUIRE((lower.optimal_normal + Vector3d::UnitZ()).norm() < 1e-12);
    REQUIRE((upper.target_position - data.target_thickness * Vector3d::UnitZ()).norm() < 1e-12);
    REQUIRE((lower.target_position + data.target_thickness * Vector3d::UnitZ()).norm() < 1e-12);
    REQUIRE(data.target_positions.row(3) == data.target_positions.row(5));
    REQUIRE(data.optimal_normals.row(3) == data.optimal_normals.row(5));
    REQUIRE(data.singular_vertex_tags[3] == 0);
    REQUIRE(data.singular_vertex_tags[4] == 0);
    const wmtk::MatrixXd targets = data.target_positions;
    evaluate_target_positions(data, 0.6);
    REQUIRE(data.offset_components.size() == 2); // no accumulated stale components
    REQUIRE((data.target_positions.row(3) - 2 * targets.row(3)).norm() < 1e-12);
    data.vertices *= 10;
    evaluate_target_positions(data, 0.3);
    REQUIRE((data.target_positions.row(3) - 10 * targets.row(3)).norm() < 1e-10);
}

TEST_CASE(
    "components cannot connect through input vertices or vertices with different correspondence")
{
    auto data = sheet(true);
    evaluate_target_positions(data, 0.1);
    REQUIRE(data.input_to_components[0].size() == 3); // two upper components, one lower
    REQUIRE(data.input_to_components[1].size() == 1);
    REQUIRE(data.vertex_component_ids[3] == data.vertex_component_ids[5]);
    REQUIRE(
        data.vertex_component_ids[5] != data.vertex_component_ids[7]); // bridge via 6 is forbidden
    REQUIRE(data.vertex_component_ids[5] != data.vertex_component_ids[6]);
    for (const auto& c : data.offset_components) REQUIRE_FALSE(c.singular);
    const auto& other_input = data.offset_components[data.vertex_component_ids[6]];
    REQUIRE(other_input.input_vertex == 1);
    REQUIRE(
        (other_input.target_position -
         (Vector3d(1, 0, 0) + data.target_thickness * Vector3d::UnitZ()))
            .norm() < 1e-12);
}

TEST_CASE("a component joining both sides has conflicting normals and remains unmoved")
{
    auto data = sheet();
    std::vector<std::array<size_t, 4>> tets = {{0, 1, 2, 3}, {0, 2, 1, 4}, {0, 1, 3, 4}};
    data.vertices.row(3) << 0.1, -0.1, 1;
    data.vertices.row(4) << 0.1, -0.1, -1;
    data.mesh = std::make_unique<wmtk::TetMesh>();
    data.mesh->init_with_isolated_vertices(8, tets);
    evaluate_target_positions(data, 0.1);
    REQUIRE(data.offset_components.size() == 1);
    REQUIRE(data.offset_components[0].singular);
    REQUIRE(data.offset_components[0].singular_reason == "no_valid_least_squares_direction");
    REQUIRE(data.singular_vertex_tags[3] == 1);
    REQUIRE(data.singular_vertex_tags[4] == 1);
    REQUIRE(data.optimal_normals.isZero());
    REQUIRE(data.target_positions == data.vertices);
}

TEST_CASE("degenerate input faces produce singular components and invalid thickness is rejected")
{
    auto data = sheet();
    data.vertices.row(2) << 2, 0, 0;
    evaluate_target_positions(data, 0.1);
    for (const auto& c : data.offset_components) {
        REQUIRE(c.singular);
        REQUIRE(c.singular_reason == "degenerate_input_face_or_tet");
    }
    REQUIRE(data.target_positions == data.vertices);
    REQUIRE_THROWS(evaluate_target_positions(data, 0));
    REQUIRE_THROWS(evaluate_target_positions(data, -0.1));
    REQUIRE_THROWS(evaluate_target_positions(data, std::numeric_limits<double>::infinity()));
}
