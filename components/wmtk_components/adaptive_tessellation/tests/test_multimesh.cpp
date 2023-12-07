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
#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>

#include <wmtk/utils/Logger.hpp>

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
    std::vector<std::shared_ptr<EdgeMesh>> edge_meshes;
    for (long tag : tags) {
        edge_meshes.emplace_back(std::static_pointer_cast<EdgeMesh>(
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                uv_mesh,
                "edge_tag",
                tag,
                PrimitiveType::Edge)));
    }
    std::map<EdgeMesh*, EdgeMesh*> sibling_edge_meshes = map_sibling_edge_meshes(edge_meshes);
    REQUIRE(sibling_edge_meshes.size() == 2);
    // Iterate through the keys using an iterator
    for (auto it = sibling_edge_meshes.begin(); it != sibling_edge_meshes.end(); ++it) {
        EdgeMesh* sibling0 = it->first;
        EdgeMesh* sibling1 = it->second;

        REQUIRE(position_mesh.map(*sibling0, seam).size() == 1);
        REQUIRE(position_mesh.map(*sibling1, seam).size() == 1);
    }
}

TEST_CASE("split_at_midpoint_multimesh")
{
    std::shared_ptr<DEBUG_TriMesh> position_mesh_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_with_3dpos());

    DEBUG_TriMesh& position_mesh = *position_mesh_ptr;

    std::shared_ptr<DEBUG_TriMesh> uv_mesh_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01_with_2duv());

    DEBUG_TriMesh& uv_mesh = *uv_mesh_ptr;

    auto uv_mesh_map = wmtk::multimesh::same_simplex_dimension_bijection(position_mesh, uv_mesh);
    position_mesh.register_child_mesh(uv_mesh_ptr, uv_mesh_map);

    Simplex seam = Simplex::edge(position_mesh.edge_tuple_from_vids(0, 1));
    auto uv_seam = position_mesh.map(uv_mesh, seam);
    REQUIRE(uv_seam.size() == 2);

    std::set<long> critical_vids = {0, 1, 2, 5, 6};
    auto tags = wmtk::multimesh::utils::create_tags(uv_mesh, critical_vids);
    REQUIRE(tags.size() == 5);
    std::vector<std::shared_ptr<EdgeMesh>> edge_meshes;
    for (long tag : tags) {
        edge_meshes.emplace_back(std::static_pointer_cast<EdgeMesh>(
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                uv_mesh,
                "edge_tag",
                tag,
                PrimitiveType::Edge)));
    }
    std::map<EdgeMesh*, EdgeMesh*> sibling_edge_meshes_map = map_sibling_edge_meshes(edge_meshes);
    REQUIRE(sibling_edge_meshes_map.size() == 2);

    auto uv_handle = uv_mesh.get_attribute_handle<double>("2duv", PrimitiveType::Vertex);
    auto position_handle =
        position_mesh.get_attribute_handle<double>("3dposition", PrimitiveType::Vertex);

    AT_op::internal::ATData op(
        uv_mesh,
        position_mesh,
        edge_meshes,
        sibling_edge_meshes_map,
        uv_handle,
        position_handle);

    wmtk::operations::OperationSettings<AT_op::ATInteriorSplitAtMidpoint> settings(op);
    settings.initialize_invariants(uv_mesh);
    // assert(settings.are_invariants_initialized());
    assert(settings.m_AT_data.uv_mesh_ptr());
    assert(settings.m_AT_data.position_mesh_ptr());
    assert(settings.edge_split_midpoint_settings.are_invariants_initialized());
    assert(settings.m_AT_data.m_position_handle.is_valid());

    Scheduler scheduler(uv_mesh);
    scheduler.add_operation_type<AT_op::ATInteriorSplitAtMidpoint>("split_interior", settings);
    assert(op.position_mesh_ptr());
    assert(op.position_mesh_ptr()->shared_from_this());
    scheduler.run_operation_on_all(PrimitiveType::Edge, "split_interior");


    REQUIRE(position_mesh.capacity(PrimitiveType::Vertex) == 6);

    // Accessor<double> pos_accessor = position_mesh.create_accessor(position_handle);
    // for (auto& p : position_mesh.get_all(PrimitiveType::Vertex)) {
    //     wmtk::logger().info("pos : {}", pos_accessor.vector_attribute(p));
    // }

    // std::vector<Tuple> vs = uv_mesh.get_all(PrimitiveType::Vertex);
    // Accessor<double> uv_accessor = uv_mesh.create_accessor(uv_handle);
    // for (auto& v : vs) {
    //     spdlog::info("v: {}", uv_accessor.vector_attribute(v));
    // }
    REQUIRE(uv_mesh.capacity(PrimitiveType::Vertex) == 8);
}
