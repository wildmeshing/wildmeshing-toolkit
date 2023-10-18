#include <catch2/catch_test_macros.hpp>

#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"
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


namespace {} // namespace


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

    const auto& p_mul_manager = parent.multi_mesh_manager();
    const auto& c0_mul_manager = child0.multi_mesh_manager();
    const auto& c1_mul_manager = child1.multi_mesh_manager();
    REQUIRE(p_mul_manager.children().size() == 2);
    REQUIRE(p_mul_manager.children()[0].mesh == child0_ptr);
    REQUIRE(p_mul_manager.children()[1].mesh == child1_ptr);
    REQUIRE(c0_mul_manager.children().size() == 0);
    REQUIRE(c1_mul_manager.children().size() == 0);
    REQUIRE(&c0_mul_manager.get_root_mesh(child0) == &parent);
    REQUIRE(&c1_mul_manager.get_root_mesh(child1) == &parent);

    REQUIRE(p_mul_manager.is_root());
    REQUIRE(!c0_mul_manager.is_root());
    REQUIRE(!c1_mul_manager.is_root());


    // test id computation
    REQUIRE(parent.absolute_multi_mesh_id().empty());
    REQUIRE(child0.absolute_multi_mesh_id() == std::vector<long>{{0}});
    REQUIRE(child1.absolute_multi_mesh_id() == std::vector<long>{{1}});

    // test attribute contents
    {
        const std::string c_to_p_name =
            DEBUG_MultiMeshManager::child_to_parent_map_attribute_name();
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

        {
            std::vector<std::tuple<Tuple, Tuple>> p_to_c0_map{
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()},
                std::tuple<Tuple, Tuple>{parent.tuple_from_id(PF, 2), child0.tuple_from_id(PF, 0)}};

            std::vector<std::tuple<Tuple, Tuple>> p_to_c1_map{
                std::tuple<Tuple, Tuple>{parent.tuple_from_id(PF, 0), child1.tuple_from_id(PF, 0)},
                std::tuple<Tuple, Tuple>{parent.tuple_from_id(PF, 1), child1.tuple_from_id(PF, 1)},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()},
            };

            std::vector<std::tuple<Tuple, Tuple>> c0_to_p_map{
                std::tuple<Tuple, Tuple>{child0.tuple_from_id(PF, 0), parent.tuple_from_id(PF, 2)}};

            std::vector<std::tuple<Tuple, Tuple>> c1_to_p_map{
                std::tuple<Tuple, Tuple>{child1.tuple_from_id(PF, 0), parent.tuple_from_id(PF, 0)},
                std::tuple<Tuple, Tuple>{child1.tuple_from_id(PF, 1), parent.tuple_from_id(PF, 1)}};


            for (long parent_index = 0; parent_index < 3; ++parent_index) {
                auto ptuple = parent.tuple_from_id(PF, parent_index);
                auto p_to_c0_tuple_tuple =
                    multimesh::utils::read_tuple_map_attribute(parent_to_child0_acc, ptuple);
                auto p_to_c1_tuple_tuple =
                    multimesh::utils::read_tuple_map_attribute(parent_to_child1_acc, ptuple);

                CHECK(p_to_c0_tuple_tuple == p_to_c0_map[parent_index]);
                CHECK(p_to_c1_tuple_tuple == p_to_c1_map[parent_index]);
            }
            for (size_t child0_index = 0; child0_index < c0_to_p_map.size(); ++child0_index) {
                auto tuple = child0.tuple_from_id(PF, child0_index);
                auto c0_to_p_tuple_tuple =
                    multimesh::utils::read_tuple_map_attribute(child0_to_parent_acc, tuple);

                CHECK(c0_to_p_tuple_tuple == c0_to_p_map[child0_index]);
            }
            for (size_t child1_index = 0; child1_index < c1_to_p_map.size(); ++child1_index) {
                auto tuple = child1.tuple_from_id(PF, child1_index);
                auto c1_to_p_tuple_tuple =
                    multimesh::utils::read_tuple_map_attribute(child1_to_parent_acc, tuple);

                CHECK(c1_to_p_tuple_tuple == c1_to_p_map[child1_index]);
            }
        }
        {
            std::vector<Tuple> p_to_c0_map{Tuple(), Tuple(), child0.tuple_from_id(PF, 0)};

            std::vector<Tuple> p_to_c1_map{
                child1.tuple_from_id(PF, 0),
                child1.tuple_from_id(PF, 1),
                Tuple()};

            std::vector<Tuple> c0_to_p_map{parent.tuple_from_id(PF, 2)};

            std::vector<Tuple> c1_to_p_map{
                parent.tuple_from_id(PF, 0),
                parent.tuple_from_id(PF, 1)};


            for (long parent_index = 0; parent_index < 3; ++parent_index) {
                auto ptuple = parent.tuple_from_id(PF, parent_index);
                Simplex psimplex = Simplex(PF, ptuple);

                Tuple c0_expected = p_to_c0_map[parent_index];
                if (!c0_expected.is_null()) {
                    auto c0tuples = parent.map_to_child_tuples(child0, psimplex);
                    REQUIRE(c0tuples.size() == 1);
                    CHECK(c0tuples[0] == c0_expected);
                }

                Tuple c1_expected = p_to_c1_map[parent_index];
                if (!c1_expected.is_null()) {
                    auto c1tuples = parent.map_to_child_tuples(child1, psimplex);
                    REQUIRE(c1tuples.size() == 1);
                    CHECK(c1tuples[0] == c1_expected);
                }
            }
            for (size_t child0_index = 0; child0_index < c0_to_p_map.size(); ++child0_index) {
                auto tuple = child0.tuple_from_id(PF, child0_index);
                Simplex csimplex = Simplex(PF, tuple);
                auto ptuple = child0.map_to_parent_tuple(csimplex);
                CHECK(ptuple == c0_to_p_map[child0_index]);
            }
            for (size_t child1_index = 0; child1_index < c1_to_p_map.size(); ++child1_index) {
                auto tuple = child1.tuple_from_id(PF, child1_index);
                Simplex csimplex = Simplex(PF, tuple);
                auto ptuple = child1.map_to_parent_tuple(csimplex);
                CHECK(ptuple == c1_to_p_map[child1_index]);
            }
        }
    }

    // test actual api calls
    {
        // try the tuples that should succeed
        for (const auto& [ct, pt] : child0_map) {
            auto ncts = parent.map_to_child_tuples(child0, Simplex(PrimitiveType::Face, pt));
            REQUIRE(ncts.size() == 1);
            auto nct = ncts[0];
            auto npt = child0.map_to_parent_tuple(Simplex(PrimitiveType::Face, ct));

            CHECK(nct == ct);
            CHECK(npt == pt);
        }
        for (const auto& [ct, pt] : child1_map) {
            auto ncts = parent.map_to_child_tuples(child1, Simplex(PrimitiveType::Face, pt));
            REQUIRE(ncts.size() == 1);
            auto nct = ncts[0];
            auto npt = child1.map_to_parent_tuple(Simplex(PrimitiveType::Face, ct));

            CHECK(nct == ct);
            CHECK(npt == pt);
        }


        // go through simplex indices that aren't available in the map
        for (long index = 0; index < 2; ++index) {
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


    p_mul_manager.check_map_valid(parent);
}

TEST_CASE("test_multi_mesh_navigation", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::shared_ptr<DEBUG_TriMesh> child2_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());

    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;
    auto& child2 = *child2_ptr;

    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {0});
    auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0, 1});
    auto child2_map = multimesh::same_simplex_dimension_surjection(parent, child2, {0, 1, 2});

    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);
    parent.register_child_mesh(child2_ptr, child2_map);


    auto get_single_child_tuple = [&](const auto& mesh, const auto& tuple) -> Tuple {
        auto tups = parent.map_to_child_tuples(mesh, Simplex(PF, tuple));
        REQUIRE(tups.size() == 1);
        return tups[0];
    };

    Tuple edge = parent.edge_tuple_between_v1_v2(1, 0, 0);
    Tuple edge_child0 = get_single_child_tuple(child0, edge);
    Tuple edge_child1 = get_single_child_tuple(child1, edge);
    Tuple edge_child2 = get_single_child_tuple(child2, edge);

    CHECK(edge_child0 == child0.edge_tuple_between_v1_v2(1, 0, 0));
    CHECK(edge_child1 == child1.edge_tuple_between_v1_v2(1, 0, 0));
    CHECK(edge_child2 == child2_ptr->edge_tuple_between_v1_v2(1, 0, 0));

    for (PrimitiveType pt : {PV, PE}) {
        CHECK(
            child0.switch_tuple(edge_child0, pt) ==
            get_single_child_tuple(child0, parent.switch_tuple(edge, pt)));
        CHECK(
            child1.switch_tuple(edge_child1, pt) ==
            get_single_child_tuple(child1, parent.switch_tuple(edge, pt)));
        CHECK(
            child2.switch_tuple(edge_child2, pt) ==
            get_single_child_tuple(child2, parent.switch_tuple(edge, pt)));
    }
}

TEST_CASE("multi_mesh_register_between_2D_and_1D", "[multimesh][1D][2D]")
{
    DEBUG_TriMesh parent = single_triangle();
    std::shared_ptr<DEBUG_EdgeMesh> child0_ptr = std::make_shared<DEBUG_EdgeMesh>(single_line());
    std::shared_ptr<DEBUG_EdgeMesh> child1_ptr = std::make_shared<DEBUG_EdgeMesh>(two_segments());

    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;

    std::vector<std::array<Tuple, 2>> child0_map(1);
    std::vector<std::array<Tuple, 2>> child1_map(2);

    child0_map[0] = {child0.tuple_from_edge_id(0), parent.tuple_from_id(PE, 0)};
    child1_map[0] = {child1.tuple_from_edge_id(0), parent.tuple_from_id(PE, 0)};
    child1_map[1] = {child1.tuple_from_edge_id(1), parent.tuple_from_id(PE, 2)};

    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);

    const auto& p_mul_manager = parent.multi_mesh_manager();
    const auto& c0_mul_manager = child0.multi_mesh_manager();
    const auto& c1_mul_manager = child1.multi_mesh_manager();
    REQUIRE(p_mul_manager.children().size() == 2);
    REQUIRE(p_mul_manager.children()[0].mesh == child0_ptr);
    REQUIRE(p_mul_manager.children()[1].mesh == child1_ptr);
    REQUIRE(c0_mul_manager.children().size() == 0);
    REQUIRE(c1_mul_manager.children().size() == 0);
    REQUIRE(&c0_mul_manager.get_root_mesh(child0) == &parent);
    REQUIRE(&c1_mul_manager.get_root_mesh(child1) == &parent);

    REQUIRE(p_mul_manager.is_root());
    REQUIRE(!c0_mul_manager.is_root());
    REQUIRE(!c1_mul_manager.is_root());

    // test id computation
    REQUIRE(parent.absolute_multi_mesh_id().empty());
    REQUIRE(child0.absolute_multi_mesh_id() == std::vector<long>{{0}});
    REQUIRE(child1.absolute_multi_mesh_id() == std::vector<long>{{1}});

    // test attribute contents
    {
        const std::string c_to_p_name =
            DEBUG_MultiMeshManager::child_to_parent_map_attribute_name();
        const std::string p_to_c0_name =
            DEBUG_MultiMeshManager::parent_to_child_map_attribute_name(0);
        const std::string p_to_c1_name =
            DEBUG_MultiMeshManager::parent_to_child_map_attribute_name(1);
        REQUIRE(parent.has_attribute<long>(p_to_c0_name, PE));
        REQUIRE(parent.has_attribute<long>(p_to_c1_name, PE));
        REQUIRE(child0.has_attribute<long>(c_to_p_name, PE));
        REQUIRE(child1.has_attribute<long>(c_to_p_name, PE));

        auto parent_to_child0_handle = parent.get_attribute_handle<long>(p_to_c0_name, PE);
        auto parent_to_child1_handle = parent.get_attribute_handle<long>(p_to_c1_name, PE);
        auto child0_to_parent_handle = child0.get_attribute_handle<long>(c_to_p_name, PE);
        auto child1_to_parent_handle = child1.get_attribute_handle<long>(c_to_p_name, PE);
        auto parent_to_child0_acc = parent.create_const_accessor(parent_to_child0_handle);
        auto parent_to_child1_acc = parent.create_const_accessor(parent_to_child1_handle);
        auto child0_to_parent_acc = child0.create_const_accessor(child0_to_parent_handle);
        auto child1_to_parent_acc = child1.create_const_accessor(child1_to_parent_handle);

        // test read_tuple_map_attribute
        {
            std::vector<std::tuple<Tuple, Tuple>> p_to_c0_map{
                std::tuple<Tuple, Tuple>{parent.tuple_from_id(PE, 0), child0.tuple_from_id(PE, 0)},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()}};

            std::vector<std::tuple<Tuple, Tuple>> p_to_c1_map{
                std::tuple<Tuple, Tuple>{parent.tuple_from_id(PE, 0), child1.tuple_from_id(PE, 0)},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()},
                std::tuple<Tuple, Tuple>{parent.tuple_from_id(PE, 2), child1.tuple_from_id(PE, 1)},
            };

            std::vector<std::tuple<Tuple, Tuple>> c0_to_p_map{
                std::tuple<Tuple, Tuple>{child0.tuple_from_id(PE, 0), parent.tuple_from_id(PE, 0)}};

            std::vector<std::tuple<Tuple, Tuple>> c1_to_p_map{
                std::tuple<Tuple, Tuple>{child1.tuple_from_id(PE, 0), parent.tuple_from_id(PE, 0)},
                std::tuple<Tuple, Tuple>{child1.tuple_from_id(PE, 1), parent.tuple_from_id(PE, 2)}};


            for (long parent_index = 0; parent_index < 3; ++parent_index) {
                auto ptuple = parent.tuple_from_id(PE, parent_index);
                auto p_to_c0_tuple_tuple =
                    multimesh::utils::read_tuple_map_attribute(parent_to_child0_acc, ptuple);
                auto p_to_c1_tuple_tuple =
                    multimesh::utils::read_tuple_map_attribute(parent_to_child1_acc, ptuple);

                CHECK(p_to_c0_tuple_tuple == p_to_c0_map[parent_index]);
                CHECK(p_to_c1_tuple_tuple == p_to_c1_map[parent_index]);
            }
            for (size_t child0_index = 0; child0_index < c0_to_p_map.size(); ++child0_index) {
                auto tuple = child0.tuple_from_id(PE, child0_index);
                auto c0_to_p_tuple_tuple =
                    multimesh::utils::read_tuple_map_attribute(child0_to_parent_acc, tuple);

                CHECK(c0_to_p_tuple_tuple == c0_to_p_map[child0_index]);
            }
            for (size_t child1_index = 0; child1_index < c1_to_p_map.size(); ++child1_index) {
                auto tuple = child1.tuple_from_id(PE, child1_index);
                auto c1_to_p_tuple_tuple =
                    multimesh::utils::read_tuple_map_attribute(child1_to_parent_acc, tuple);

                CHECK(c1_to_p_tuple_tuple == c1_to_p_map[child1_index]);
            }
        }

        // test map_to_child_tuples and map_to_parent_tuple
        // TODO: this will fail before getting local_switch_tuple for EdgeMesh Working.
        // {
        //     std::vector<Tuple> p_to_c0_map{child0.tuple_from_id(PE, 0), Tuple(), Tuple()};

        //     std::vector<Tuple> p_to_c1_map{
        //         child1.tuple_from_id(PE, 0),
        //         Tuple(),
        //         child1.tuple_from_id(PE, 1)};

        //     std::vector<Tuple> c0_to_p_map{parent.tuple_from_id(PE, 0)};

        //     std::vector<Tuple> c1_to_p_map{
        //         parent.tuple_from_id(PE, 0),
        //         parent.tuple_from_id(PE, 2)};


        //     for (long parent_index = 0; parent_index < 3; ++parent_index) {
        //         auto ptuple = parent.tuple_from_id(PE, parent_index);
        //         Simplex psimplex = Simplex(PE, ptuple);

        //         Tuple c0_expected = p_to_c0_map[parent_index];
        //         if (!c0_expected.is_null()) {
        //             auto c0tuples = parent.map_to_child_tuples(child0, psimplex);
        //             REQUIRE(c0tuples.size() == 1);
        //             CHECK(c0tuples[0] == c0_expected);
        //         }

        //         Tuple c1_expected = p_to_c1_map[parent_index];
        //         if (!c1_expected.is_null()) {
        //             auto c1tuples = parent.map_to_child_tuples(child1, psimplex);
        //             REQUIRE(c1tuples.size() == 1);
        //             CHECK(c1tuples[0] == c1_expected);
        //         }
        //     }
        //     for (size_t child0_index = 0; child0_index < c0_to_p_map.size(); ++child0_index) {
        //         auto tuple = child0.tuple_from_id(PE, child0_index);
        //         Simplex csimplex = Simplex(PE, tuple);
        //         auto ptuple = child0.map_to_parent_tuple(csimplex);
        //         CHECK(ptuple == c0_to_p_map[child0_index]);
        //     }
        //     for (size_t child1_index = 0; child1_index < c1_to_p_map.size(); ++child1_index) {
        //         auto tuple = child1.tuple_from_id(PE, child1_index);
        //         Simplex csimplex = Simplex(PE, tuple);
        //         auto ptuple = child1.map_to_parent_tuple(csimplex);
        //         CHECK(ptuple == c1_to_p_map[child1_index]);
        //     }
        // }
    }
}

/*
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
