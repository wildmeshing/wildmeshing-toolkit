#include <catch2/catch_test_macros.hpp>

#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
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


namespace {
class DEBUG_MultiMeshManager : public MultiMeshManager
{
public:
    using MultiMeshManager::child_to_parent_map_attribute_name;
    using MultiMeshManager::children;
    using MultiMeshManager::parent_to_child_map_attribute_name;
};
} // namespace


TEST_CASE("test_register_child_mesh", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());


    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;

    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {2});
    auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0, 1});

    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);

    const auto& p_mul_manager =
        reinterpret_cast<const DEBUG_MultiMeshManager&>(parent.multi_mesh_manager());
    REQUIRE(p_mul_manager.children().size() == 2);
    REQUIRE(p_mul_manager.children()[0].mesh == child0_ptr);
    REQUIRE(p_mul_manager.children()[1].mesh == child1_ptr);


    // test id computation
    REQUIRE(parent.absolute_multi_mesh_id().empty());
    REQUIRE(child0.absolute_multi_mesh_id() == std::vector<long>{{0}});
    REQUIRE(child1.absolute_multi_mesh_id() == std::vector<long>{{1}});

    // test attribute contents
    {
        const std::string c_to_p_name =
            DEBUG_MultiMeshManager::child_to_parent_map_attribute_name();
        ;
        const std::string p_to_c0_name =
            DEBUG_MultiMeshManager::parent_to_child_map_attribute_name(0);
        const std::string p_to_c1_name =
            DEBUG_MultiMeshManager::parent_to_child_map_attribute_name(1);
        REQUIRE(parent.has_attribute<long>(p_to_c0_name, PF));
        REQUIRE(parent.has_attribute<long>(p_to_c1_name, PF));
        REQUIRE(child0.has_attribute<long>(c_to_p_name, PF));
        REQUIRE(child1.has_attribute<long>(c_to_p_name, PF));

        auto parent_to_child0_handle = parent.get_attribute_handle<long>(p_to_c0_name, PF);
        auto parent_to_child1_handle = parent.get_attribute_handle<long>(p_to_c1_name, PF);
        auto child0_to_parent_handle = child0.get_attribute_handle<long>(c_to_p_name, PF);
        auto child1_to_parent_handle = child1.get_attribute_handle<long>(c_to_p_name, PF);

        auto parent_to_child0_acc = parent.create_const_accessor(parent_to_child0_handle);
        auto parent_to_child1_acc = parent.create_const_accessor(parent_to_child1_handle);
        auto child0_to_parent_acc = child0.create_const_accessor(child0_to_parent_handle);
        auto child1_to_parent_acc = child1.create_const_accessor(child1_to_parent_handle);
    }

    // test actual api calls
    {
        // try the tuples that should succeed
        for (const auto& [pt, ct] : child0_map) {
            auto ncts = parent.map_to_child_tuples(child0, Simplex(PrimitiveType::Face, pt));
            REQUIRE(ncts.size() == 1);
            auto nct = ncts[0];
            auto npt = child0.map_to_parent_tuple(Simplex(PrimitiveType::Face, ct));

            CHECK(nct == ct);
            CHECK(npt == pt);
        }
        for (const auto& [pt, ct] : child1_map) {
            auto ncts = parent.map_to_child_tuples(child1, Simplex(PrimitiveType::Face, pt));
            REQUIRE(ncts.size() == 1);
            auto nct = ncts[0];
            auto npt = child1.map_to_parent_tuple(Simplex(PrimitiveType::Face, ct));

            CHECK(nct == ct);
            CHECK(npt == pt);
        }


        // go through simplex indices that aren't available in the map
        for (long index = 1; index < 3; ++index) {
            auto pt = parent.tuple_from_id(PF, index);
            auto ncts = parent.map_to_child(child0, Simplex(PrimitiveType::Face, pt));
            CHECK(ncts.size() == 0);
        }
        for (long index = 2; index < 3; ++index) {
            auto pt = parent.tuple_from_id(PF, index);
            auto ncts = parent.map_to_child(child1, Simplex(PrimitiveType::Face, pt));
            CHECK(ncts.size() == 0);
        }
    }


    // REQUIRE(p_mul_manager.is_map_valid(parent) == true);
}

/*
TEST_CASE("test_multi_mesh_navigation", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr =
std::make_shared<DEBUG_TriMesh>(single_triangle()); std::vector<long> child0_map = {0};
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::vector<long> child1_map = {0, 1};
    std::shared_ptr<DEBUG_TriMesh> child2_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());
    std::vector<long> child2_map = {0, 1, 2};

    MultiMeshManager::register_child_mesh(parent, child0_ptr, child0_map);
    MultiMeshManager::register_child_mesh(parent, child1_ptr, child1_map);
    MultiMeshManager::register_child_mesh(parent, child2_ptr, child2_map);

    Tuple edge = parent.edge_tuple_between_v1_v2(1, 0, 0);
    Tuple edge_child0 = MultiMeshManager::map_tuple_between_meshes(
        parent,
        *child0_ptr,
        parent.multi_mesh_manager.map_to_child_handles[0],
        edge);
    Tuple edge_child1 = MultiMeshManager::map_tuple_between_meshes(
        parent,
        *child1_ptr,
        parent.multi_mesh_manager.map_to_child_handles[1],
        edge);
    Tuple edge_child2 = MultiMeshManager::map_tuple_between_meshes(
        parent,
        *child2_ptr,
        parent.multi_mesh_manager.map_to_child_handles[2],
        edge);

    CHECK(edge_child0 == child0.edge_tuple_between_v1_v2(1, 0, 0));
    CHECK(edge_child1 == child1.edge_tuple_between_v1_v2(1, 0, 0));
    CHECK(edge_child2 == child2_ptr->edge_tuple_between_v1_v2(1, 0, 0));

    CHECK(
        child0.switch_vertex(edge_child0) ==
        MultiMeshManager::map_tuple_between_meshes(
            parent,
            *child0_ptr,
            parent.multi_mesh_manager.map_to_child_handles[0],
            parent.switch_vertex(edge)));
    CHECK(
        child1.switch_vertex(edge_child1) ==
        MultiMeshManager::map_tuple_between_meshes(
            parent,
            *child1_ptr,
            parent.multi_mesh_manager.map_to_child_handles[1],
            parent.switch_vertex(edge)));
    CHECK(
        child2_ptr->switch_vertex(edge_child2) ==
        MultiMeshManager::map_tuple_between_meshes(
            parent,
            *child2_ptr,
            parent.multi_mesh_manager.map_to_child_handles[2],
            parent.switch_vertex(edge)));

    CHECK(
        child0.switch_edge(edge_child0) ==
        MultiMeshManager::map_tuple_between_meshes(
            parent,
            *child0_ptr,
            parent.multi_mesh_manager.map_to_child_handles[0],
            parent.switch_edge(edge)));
    CHECK(
        child1.switch_edge(edge_child1) ==
        MultiMeshManager::map_tuple_between_meshes(
            parent,
            *child1_ptr,
            parent.multi_mesh_manager.map_to_child_handles[1],
            parent.switch_edge(edge)));
    CHECK(
        child2_ptr->switch_edge(edge_child2) ==
        MultiMeshManager::map_tuple_between_meshes(
            parent,
            *child2_ptr,
            parent.multi_mesh_manager.map_to_child_handles[2],
            parent.switch_edge(edge)));
}

TEST_CASE("test_split_multi_mesh", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr =
std::make_shared<DEBUG_TriMesh>(single_triangle()); std::vector<long> child0_map = {0};
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::vector<long> child1_map = {0, 1};
    std::shared_ptr<DEBUG_TriMesh> child2_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());
    std::vector<long> child2_map = {0, 1, 2};

    MultiMeshManager::register_child_mesh(parent, child0_ptr, child0_map);
    MultiMeshManager::register_child_mesh(parent, child1_ptr, child1_map);
    MultiMeshManager::register_child_mesh(parent, child2_ptr, child2_map);
    REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

    Tuple edge = parent.edge_tuple_between_v1_v2(1, 0, 1);
    auto parent_hash_acc = parent.get_cell_hash_accessor();
    auto executor = parent.get_tmoe(edge, parent_hash_acc);
    executor.split_edge();

    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0.is_connectivity_valid());
    REQUIRE(child1.is_connectivity_valid());
    REQUIRE(child2_ptr->is_connectivity_valid());
    REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

    CHECK(parent.fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(parent.fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(5, 1, 2));
    CHECK(parent.fv_from_fid(4) == Eigen::Matrix<long, 3, 1>(0, 5, 2));
    CHECK(parent.fv_from_fid(5) == Eigen::Matrix<long, 3, 1>(3, 1, 5));
    CHECK(parent.fv_from_fid(6) == Eigen::Matrix<long, 3, 1>(3, 5, 0));
    CHECK(child0.fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 1, 2));
    CHECK(child0.fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 3, 2));
    CHECK(child1.fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(4, 1, 2));
    CHECK(child1.fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(0, 4, 2));
    CHECK(child1.fv_from_fid(4) == Eigen::Matrix<long, 3, 1>(3, 1, 4));
    CHECK(child1.fv_from_fid(5) == Eigen::Matrix<long, 3, 1>(3, 4, 0));
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
    REQUIRE(child0.is_connectivity_valid());
    REQUIRE(child1.is_connectivity_valid());
    REQUIRE(child2_ptr->is_connectivity_valid());
    REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

    CHECK(parent.fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(parent.fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(5, 1, 2));
    CHECK(parent.fv_from_fid(5) == Eigen::Matrix<long, 3, 1>(3, 1, 5));
    CHECK(parent.fv_from_fid(7) == Eigen::Matrix<long, 3, 1>(0, 6, 2));
    CHECK(parent.fv_from_fid(8) == Eigen::Matrix<long, 3, 1>(6, 5, 2));
    CHECK(parent.fv_from_fid(9) == Eigen::Matrix<long, 3, 1>(3, 6, 0));
    CHECK(parent.fv_from_fid(10) == Eigen::Matrix<long, 3, 1>(3, 5, 6));

    CHECK(child0.fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 1, 2));
    CHECK(child0.fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(0, 4, 2));
    CHECK(child0.fv_from_fid(4) == Eigen::Matrix<long, 3, 1>(4, 3, 2));

    CHECK(child1.fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(4, 1, 2));
    CHECK(child1.fv_from_fid(4) == Eigen::Matrix<long, 3, 1>(3, 1, 4));
    CHECK(child1.fv_from_fid(6) == Eigen::Matrix<long, 3, 1>(0, 5, 2));
    CHECK(child1.fv_from_fid(7) == Eigen::Matrix<long, 3, 1>(5, 4, 2));
    CHECK(child1.fv_from_fid(8) == Eigen::Matrix<long, 3, 1>(3, 5, 0));
    CHECK(child1.fv_from_fid(9) == Eigen::Matrix<long, 3, 1>(3, 4, 5));

    CHECK(child2_ptr->fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(child2_ptr->fv_from_fid(3) == Eigen::Matrix<long, 3, 1>(7, 1, 2));
    CHECK(child2_ptr->fv_from_fid(5) == Eigen::Matrix<long, 3, 1>(3, 5, 8));
    CHECK(child2_ptr->fv_from_fid(7) == Eigen::Matrix<long, 3, 1>(0, 9, 2));
    CHECK(child2_ptr->fv_from_fid(8) == Eigen::Matrix<long, 3, 1>(9, 7, 2));
    CHECK(child2_ptr->fv_from_fid(9) == Eigen::Matrix<long, 3, 1>(3, 10, 6));
    CHECK(child2_ptr->fv_from_fid(10) == Eigen::Matrix<long, 3, 1>(3, 8, 10));
}

TEST_CASE("test_collapse_multi_mesh", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr =
std::make_shared<DEBUG_TriMesh>(two_neighbors()); std::vector<long> child0_map = {0, 1, 2};
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::vector<long> child1_map = {0, 1};
    std::shared_ptr<DEBUG_TriMesh> child2_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());
    std::vector<long> child2_map = {0, 1, 2};

    MultiMeshManager::register_child_mesh(parent, child0_ptr, child0_map);
    MultiMeshManager::register_child_mesh(parent, child1_ptr, child1_map);
    MultiMeshManager::register_child_mesh(parent, child2_ptr, child2_map);
    REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

    Tuple edge = parent.edge_tuple_between_v1_v2(1, 2, 0);
    auto parent_hash_acc = parent.get_cell_hash_accessor();
    auto executor = parent.get_tmoe(edge, parent_hash_acc);
    executor.collapse_edge();

    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0.is_connectivity_valid());
    REQUIRE(child1.is_connectivity_valid());
    REQUIRE(child2_ptr->is_connectivity_valid());
    REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

    CHECK(parent.fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 2, 0));
    CHECK(parent.fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(child0.fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 2, 0));
    CHECK(child0.fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
    CHECK(child1.fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 2, 0));
    CHECK(child2_ptr->fv_from_fid(1) == Eigen::Matrix<long, 3, 1>(3, 5, 6));
    CHECK(child2_ptr->fv_from_fid(2) == Eigen::Matrix<long, 3, 1>(0, 2, 4));
}

*/
