#include <cassert>
#include <catch2/catch_test_macros.hpp>
#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/DEBUG_Tuple.hpp>
#include <tools/EdgeMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/utils/create_tag.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include "wmtk/multimesh/utils/map_sibling_edge_meshes.hpp"

#include <wmtk/Scheduler.hpp>
#include <wmtk/components/adaptive_tessellation/operations/ATInteriorSplitAtMidpoint.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATOperation.hpp>

using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::components::adaptive_tessellation::multimesh::utils;
namespace AT_op = wmtk::components::adaptive_tessellation::operations;
TEST_CASE("edge mesh registration")
{
    DEBUG_TriMesh position_mesh = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> uv_mesh_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());

    auto& uv_mesh = *uv_mesh_ptr;

    auto uv_mesh_map = wmtk::multimesh::same_simplex_dimension_bijection(position_mesh, uv_mesh);
    position_mesh.register_child_mesh(uv_mesh_ptr, uv_mesh_map);

    Simplex seam = Simplex::edge(position_mesh.edge_tuple_from_vids(0, 1));
    auto uv_seam = position_mesh.map(uv_mesh, seam);
    REQUIRE(uv_seam.size() == 2);

    std::set<long> critical_vids = {0, 1, 2, 5, 6};
    auto tags = wmtk::multimesh::utils::create_tags(uv_mesh, critical_vids);
    REQUIRE(tags.size() == 5);
    std::vector<std::shared_ptr<Mesh>> edge_meshes;
    for (long tag : tags) {
        edge_meshes.emplace_back(wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            uv_mesh,
            "edge_tag",
            tag,
            PrimitiveType::Edge));
    }
    std::map<Mesh*, Mesh*> sibling_edge_meshes = map_sibling_edge_meshes(edge_meshes);
    REQUIRE(sibling_edge_meshes.size() == 2);
    // Iterate through the keys using an iterator
    for (auto it = sibling_edge_meshes.begin(); it != sibling_edge_meshes.end(); ++it) {
        Mesh* sibling0 = it->first;
        Mesh* sibling1 = it->second;

        REQUIRE(position_mesh.map(*sibling0, seam).size() == 1);
        REQUIRE(position_mesh.map(*sibling1, seam).size() == 1);
    }
}

TEST_CASE("split_at_midpoint_multimesh")
{
    DEBUG_TriMesh position_mesh = two_neighbors_with_3dpos();
    std::shared_ptr<DEBUG_TriMesh> uv_mesh_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01_with_2duv());

    auto& uv_mesh = *uv_mesh_ptr;

    auto uv_mesh_map = wmtk::multimesh::same_simplex_dimension_bijection(position_mesh, uv_mesh);
    position_mesh.register_child_mesh(uv_mesh_ptr, uv_mesh_map);

    Simplex seam = Simplex::edge(position_mesh.edge_tuple_from_vids(0, 1));
    auto uv_seam = position_mesh.map(uv_mesh, seam);
    REQUIRE(uv_seam.size() == 2);

    std::set<long> critical_vids = {0, 1, 2, 5, 6};
    auto tags = wmtk::multimesh::utils::create_tags(uv_mesh, critical_vids);
    REQUIRE(tags.size() == 5);
    std::vector<std::shared_ptr<Mesh>> edge_meshes;
    for (long tag : tags) {
        edge_meshes.emplace_back(wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            uv_mesh,
            "edge_tag",
            tag,
            PrimitiveType::Edge));
    }
    std::map<Mesh*, Mesh*> sibling_edge_meshes_map = map_sibling_edge_meshes(edge_meshes);
    REQUIRE(sibling_edge_meshes_map.size() == 2);

    auto uv_handle = uv_mesh.get_attribute_handle<double>("2duv", PrimitiveType::Vertex);
    auto position_handle =
        position_mesh.get_attribute_handle<double>("3dposition", PrimitiveType::Vertex);

    AT_op::internal::ATOperation op(
        uv_mesh,
        position_mesh,
        edge_meshes,
        sibling_edge_meshes_map,
        uv_handle,
        position_handle);

    wmtk::operations::OperationSettings<AT_op::ATInteriorSplitAtMidpoint> settings(op);
    settings.initialize_invariants(uv_mesh);

    Scheduler scheduler(uv_mesh);
    scheduler.add_operation_type<AT_op::ATInteriorSplitAtMidpoint>("split_interior", settings);
    scheduler.run_operation_on_all(PrimitiveType::Edge, "split_interior");

    REQUIRE(uv_mesh.capacity(PrimitiveType::Vertex) == 8);
    REQUIRE(position_mesh.capacity(PrimitiveType::Vertex) == 6);
}