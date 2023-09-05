#include <catch2/catch_test_macros.hpp>

#include <wmtk/TriMeshOperationExecutor.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;

using TM = TriMesh;
using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe(
    wmtk::Tuple(),
    std::declval<Accessor<long>&>()));

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;

TEST_CASE("test_register_child_mesh","[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::vector<long> child0_map = {0};
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::vector<long> child1_map = {0,1};

    MultiMeshManager::register_child_mesh(parent, child0_ptr, child0_map);
    MultiMeshManager::register_child_mesh(parent, child1_ptr, child1_map);

    auto p_mul_manager = parent.multi_mesh_manager;
    REQUIRE(p_mul_manager.child_meshes.size() == 2);
    REQUIRE(p_mul_manager.child_meshes[0] == child0_ptr);
    REQUIRE(p_mul_manager.child_meshes[1] == child1_ptr);

    auto [tuple1, tuple2] = MultiMeshManager::read_tuple_map_attribute(p_mul_manager.map_to_child_handles[0], parent, parent.tuple_from_id(PF,0));
    REQUIRE(tuple1 == parent.tuple_from_id(PF,0));
    REQUIRE(tuple2 == child0_ptr->tuple_from_id(PF,0));
    
    auto [tuple3, tuple4] = MultiMeshManager::read_tuple_map_attribute(p_mul_manager.map_to_child_handles[0], parent, parent.tuple_from_id(PF,1));

    REQUIRE(!tuple3.is_null());
    REQUIRE(tuple4.is_null());

    auto c2_mul_manager = child1_ptr->multi_mesh_manager;

    REQUIRE(c2_mul_manager.is_parent_mesh() == false);
    REQUIRE(c2_mul_manager.child_id() == 1);

    auto [tuple5, tuple6] = MultiMeshManager::read_tuple_map_attribute(c2_mul_manager.map_to_parent_handle, *child1_ptr, child1_ptr->tuple_from_id(PF,1));

    REQUIRE(tuple5 == child1_ptr->tuple_from_id(PF,1));
    REQUIRE(tuple6 == parent.tuple_from_id(PF,1));

    REQUIRE(p_mul_manager.is_map_valid(parent) == true);
}

TEST_CASE("test_multi_mesh_navigation","[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::vector<long> child0_map = {0};
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::vector<long> child1_map = {0,1};
    std::shared_ptr<DEBUG_TriMesh> child2_ptr = std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());
    std::vector<long> child2_map = {0,1,2};

    MultiMeshManager::register_child_mesh(parent, child0_ptr, child0_map);
    MultiMeshManager::register_child_mesh(parent, child1_ptr, child1_map);
    MultiMeshManager::register_child_mesh(parent, child2_ptr, child2_map);

    Tuple edge = parent.edge_tuple_between_v1_v2(1, 0, 0);
    Tuple edge_child0 = MultiMeshManager::map_tuple_between_meshes(parent, *child0_ptr, parent.multi_mesh_manager.map_to_child_handles[0], edge);
    Tuple edge_child1 = MultiMeshManager::map_tuple_between_meshes(parent, *child1_ptr, parent.multi_mesh_manager.map_to_child_handles[1], edge);
    Tuple edge_child2 = MultiMeshManager::map_tuple_between_meshes(parent, *child2_ptr, parent.multi_mesh_manager.map_to_child_handles[2], edge);

    CHECK(edge_child0 == child0_ptr->edge_tuple_between_v1_v2(1, 0, 0));
    CHECK(edge_child1 == child1_ptr->edge_tuple_between_v1_v2(1, 0, 0));
    CHECK(edge_child2 == child2_ptr->edge_tuple_between_v1_v2(1, 0, 0));

    CHECK(child0_ptr->switch_vertex(edge_child0) == MultiMeshManager::map_tuple_between_meshes(parent, *child0_ptr, parent.multi_mesh_manager.map_to_child_handles[0], parent.switch_vertex(edge)));
    CHECK(child1_ptr->switch_vertex(edge_child1) == MultiMeshManager::map_tuple_between_meshes(parent, *child1_ptr, parent.multi_mesh_manager.map_to_child_handles[1], parent.switch_vertex(edge)));
    CHECK(child2_ptr->switch_vertex(edge_child2) == MultiMeshManager::map_tuple_between_meshes(parent, *child2_ptr, parent.multi_mesh_manager.map_to_child_handles[2], parent.switch_vertex(edge)));

    CHECK(child0_ptr->switch_edge(edge_child0) == MultiMeshManager::map_tuple_between_meshes(parent, *child0_ptr, parent.multi_mesh_manager.map_to_child_handles[0], parent.switch_edge(edge)));
    CHECK(child1_ptr->switch_edge(edge_child1) == MultiMeshManager::map_tuple_between_meshes(parent, *child1_ptr, parent.multi_mesh_manager.map_to_child_handles[1], parent.switch_edge(edge)));
    CHECK(child2_ptr->switch_edge(edge_child2) == MultiMeshManager::map_tuple_between_meshes(parent, *child2_ptr, parent.multi_mesh_manager.map_to_child_handles[2], parent.switch_edge(edge)));
}

TEST_CASE("test_split_multi_mesh","[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::vector<long> child0_map = {0};
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::vector<long> child1_map = {0,1};
    std::shared_ptr<DEBUG_TriMesh> child2_ptr = std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());
    std::vector<long> child2_map = {0,1,2};

    MultiMeshManager::register_child_mesh(parent, child0_ptr, child0_map);
    MultiMeshManager::register_child_mesh(parent, child1_ptr, child1_map);
    MultiMeshManager::register_child_mesh(parent, child2_ptr, child2_map);
    REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

    Tuple edge = parent.edge_tuple_between_v1_v2(1, 0, 1);
    auto parent_hash_acc = parent.get_cell_hash_accessor();
    auto executor = parent.get_tmoe(edge, parent_hash_acc);
    executor.split_edge();

    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0_ptr->is_connectivity_valid());
    REQUIRE(child1_ptr->is_connectivity_valid());
    REQUIRE(child2_ptr->is_connectivity_valid());
    REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

    CHECK(parent.fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(parent.fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(5, 1, 2));
    CHECK(parent.fv_from_fid(4) == Eigen::Matrix<long, 3, 1>(0, 5, 2));
    CHECK(parent.fv_from_fid(5) == Eigen::Matrix<long, 3, 1>(3, 1, 5));
    CHECK(parent.fv_from_fid(6) == Eigen::Matrix<long, 3, 1>(3, 5, 0));
    CHECK(child0_ptr->fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 1, 2));
    CHECK(child0_ptr->fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 3, 2));
    CHECK(child1_ptr->fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(4, 1, 2));
    CHECK(child1_ptr->fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(0, 4, 2));
    CHECK(child1_ptr->fv_from_fid(4) == Eigen::Matrix<long, 3, 1>(3, 1, 4));
    CHECK(child1_ptr->fv_from_fid(5) == Eigen::Matrix<long, 3, 1>(3, 4, 0));
    CHECK(child2_ptr->fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(child2_ptr->fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(7, 1, 2));
    CHECK(child2_ptr->fv_from_fid(4) == Eigen::Matrix<long, 3, 1>(0, 7, 2));
    CHECK(child2_ptr->fv_from_fid(5) == Eigen::Matrix<long, 3, 1>(3, 5, 8));
    CHECK(child2_ptr->fv_from_fid(6) == Eigen::Matrix<long, 3, 1>(3, 8, 6));

    // Do another edge_split
    edge = parent.edge_tuple_between_v1_v2(0, 5, 4);
    auto executor1 = parent.get_tmoe(edge, parent_hash_acc);
    executor1.split_edge();

    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0_ptr->is_connectivity_valid());
    REQUIRE(child1_ptr->is_connectivity_valid());
    REQUIRE(child2_ptr->is_connectivity_valid());
    REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);  

    CHECK(parent.fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(parent.fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(5, 1, 2));
    CHECK(parent.fv_from_fid(5) == Eigen::Matrix<long, 3, 1>(3, 1, 5));
    CHECK(parent.fv_from_fid(7) == Eigen::Matrix<long, 3, 1>(0, 6, 2));
    CHECK(parent.fv_from_fid(8) == Eigen::Matrix<long, 3, 1>(6, 5, 2));
    CHECK(parent.fv_from_fid(9) == Eigen::Matrix<long, 3, 1>(3, 6, 0));
    CHECK(parent.fv_from_fid(10) == Eigen::Matrix<long, 3, 1>(3, 5, 6));
    
    CHECK(child0_ptr->fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 1, 2));
    CHECK(child0_ptr->fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(0, 4, 2));
    CHECK(child0_ptr->fv_from_fid(4) == Eigen::Matrix<long, 3, 1>(4, 3, 2));

    CHECK(child1_ptr->fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(4, 1, 2));
    CHECK(child1_ptr->fv_from_fid(4) == Eigen::Matrix<long, 3, 1>(3, 1, 4));
    CHECK(child1_ptr->fv_from_fid(6) == Eigen::Matrix<long, 3, 1>(0, 5, 2));
    CHECK(child1_ptr->fv_from_fid(7) == Eigen::Matrix<long, 3, 1>(5, 4, 2));
    CHECK(child1_ptr->fv_from_fid(8) == Eigen::Matrix<long, 3, 1>(3, 5, 0));
    CHECK(child1_ptr->fv_from_fid(9) == Eigen::Matrix<long, 3, 1>(3, 4, 5));

    CHECK(child2_ptr->fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(child2_ptr->fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(7, 1, 2));
    CHECK(child2_ptr->fv_from_fid(5) == Eigen::Matrix<long, 3, 1>(3, 5, 8));
    CHECK(child2_ptr->fv_from_fid(7) == Eigen::Matrix<long, 3, 1>(0, 9, 2));
    CHECK(child2_ptr->fv_from_fid(8) == Eigen::Matrix<long, 3, 1>(9, 7, 2));
    CHECK(child2_ptr->fv_from_fid(9) == Eigen::Matrix<long, 3, 1>(3, 10, 6));
    CHECK(child2_ptr->fv_from_fid(10) == Eigen::Matrix<long, 3, 1>(3, 8, 10));
}

TEST_CASE("test_collapse_multi_mesh","[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(two_neighbors());
    std::vector<long> child0_map = {0,1,2};
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::vector<long> child1_map = {0,1};
    std::shared_ptr<DEBUG_TriMesh> child2_ptr = std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());
    std::vector<long> child2_map = {0,1,2};

    MultiMeshManager::register_child_mesh(parent, child0_ptr, child0_map);
    MultiMeshManager::register_child_mesh(parent, child1_ptr, child1_map);
    MultiMeshManager::register_child_mesh(parent, child2_ptr, child2_map);
    REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

    Tuple edge = parent.edge_tuple_between_v1_v2(1, 2, 0);
    auto parent_hash_acc = parent.get_cell_hash_accessor();
    auto executor = parent.get_tmoe(edge, parent_hash_acc);
    executor.collapse_edge();

    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0_ptr->is_connectivity_valid());
    REQUIRE(child1_ptr->is_connectivity_valid());
    REQUIRE(child2_ptr->is_connectivity_valid());
    REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

    CHECK(parent.fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 2, 0));
    CHECK(parent.fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(child0_ptr->fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 2, 0));
    CHECK(child0_ptr->fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(child1_ptr->fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 2, 0));
    CHECK(child2_ptr->fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 5, 6));
    CHECK(child2_ptr->fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    
}


