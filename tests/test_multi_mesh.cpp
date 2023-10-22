#include <catch2/catch_test_macros.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapse.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/DEBUG_Tuple.hpp"
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
    CHECK(edge_child2 == child2.edge_tuple_between_v1_v2(1, 0, 0));

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

TEST_CASE("test_split_multi_mesh", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::shared_ptr<DEBUG_TriMesh> child2_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());

    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;
    auto& child2 = *child2_ptr;

    parent.reserve_more_attributes({10, 10, 10});
    child0.reserve_more_attributes({10, 10, 10});
    child1.reserve_more_attributes({10, 10, 10});
    child2.reserve_more_attributes({10, 10, 10});

    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {0});
    auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0, 1});
    auto child2_map = multimesh::same_simplex_dimension_surjection(parent, child2, {0, 1, 2});


    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);
    parent.register_child_mesh(child2_ptr, child2_map);

    // test id computation
    REQUIRE(parent.absolute_multi_mesh_id().empty());
    REQUIRE(child0.absolute_multi_mesh_id() == std::vector<long>{{0}});
    REQUIRE(child1.absolute_multi_mesh_id() == std::vector<long>{{1}});
    REQUIRE(child2.absolute_multi_mesh_id() == std::vector<long>{{2}});

    const auto& p_mul_manager = parent.multi_mesh_manager();
    p_mul_manager.check_map_valid(parent);

    {
        // PARENT:
        //  3-- --- 0 ---- 4
        //   |     X \     |
        //   | f1 X   \ f2 |
        //   |   X f0  \   |
        //   |  X       \  |
        //   1  ---------  2
        // vertex = 1
        // face = f1
        // (XXXX indicates the edge)
        //
        Tuple edge = parent.edge_tuple_between_v1_v2(1, 0, 1);
        simplex::Simplex edge_simplex = simplex::Simplex(PrimitiveType::Edge, edge);

        Tuple edge_f0 = parent.edge_tuple_between_v1_v2(1, 0, 0);
        simplex::Simplex edge_f0_simplex = simplex::Simplex(PrimitiveType::Edge, edge_f0);

        // CHILD0:
        //         0
        //        X \   .
        //       X   |  \ .
        //      X  0  \  \|
        //     X       \ .
        //  1  --------- 2
        {
            std::vector<simplex::Simplex> children = parent.map_to_child(child0, edge_simplex);
            REQUIRE(children.size() == 1);
            const Simplex& cs = children[0];
            REQUIRE(child0.is_valid_slow(cs.tuple()));
            REQUIRE(cs == edge_f0_simplex);
        }

        // CHILD1:
        //  3------ 0
        //   |     X \ .
        //   | f1 X   \  .
        //   |   X f0  \ .
        //   |  X       \ .
        //  1  --------- 2
        //
        {
            std::vector<simplex::Simplex> children = parent.map_to_child(child1, edge_simplex);
            REQUIRE(children.size() == 1);
            const Simplex& cs = children[0];
            REQUIRE(child1.is_valid_slow(cs.tuple()));
            REQUIRE(cs == edge_simplex);
        }

        // CHILD2:
        //  3--1--- 6
        //   |     /
        //   2 f1 0
        //   |   /
        //   |  /  ^
        //   5     |
        //         |   0 --1- 4
        //         v  / \     |
        //           /2 1\ f2 |
        //         0/ f0  \1  0
        //         /       \  |
        //      1  ----0----  2
        //
        {
            std::vector<simplex::Simplex> children = parent.map_to_child(child2, edge_simplex);
            REQUIRE(children.size() == 2);
            const Simplex& cs0 = children[0];
            const Simplex& cs1 = children[1];

            std::cout << std::string(DEBUG_Tuple(cs0.tuple())) << " "
                      << std::string(DEBUG_Tuple(cs1.tuple())) << std::endl;
            std::cout << std::string(DEBUG_Tuple(edge_f0_simplex.tuple())) << " "
                      << std::string(DEBUG_Tuple(edge_simplex.tuple())) << std::endl;

            REQUIRE(child2.is_valid_slow(cs0.tuple()));
            REQUIRE(cs0 == edge_f0_simplex);
            REQUIRE(child2.is_valid_slow(cs1.tuple()));
            REQUIRE(cs1 == edge_simplex);
        }

        operations::OperationSettings<operations::tri_mesh::EdgeSplit> settings;
        settings.initialize_invariants(parent);
        operations::tri_mesh::EdgeSplit split(parent, edge, settings);
        REQUIRE(split());
    }

    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0.is_connectivity_valid());
    REQUIRE(child1.is_connectivity_valid());
    REQUIRE(child2.is_connectivity_valid());

    CHECK(parent.fv_from_fid(2) == Vector3l(0, 2, 4));
    CHECK(parent.fv_from_fid(5) == Vector3l(5, 1, 2));
    CHECK(parent.fv_from_fid(6) == Vector3l(0, 5, 2));
    CHECK(parent.fv_from_fid(3) == Vector3l(3, 1, 5));
    CHECK(parent.fv_from_fid(4) == Vector3l(3, 5, 0));
    CHECK(child0.fv_from_fid(1) == Vector3l(3, 1, 2));
    CHECK(child0.fv_from_fid(2) == Vector3l(0, 3, 2));
    CHECK(child1.fv_from_fid(4) == Vector3l(4, 1, 2));
    CHECK(child1.fv_from_fid(5) == Vector3l(0, 4, 2));
    CHECK(child1.fv_from_fid(2) == Vector3l(3, 1, 4));
    CHECK(child1.fv_from_fid(3) == Vector3l(3, 4, 0));
    CHECK(child2.fv_from_fid(2) == Vector3l(0, 2, 4));
    CHECK(child2.fv_from_fid(3) == Vector3l(7, 1, 2));
    CHECK(child2.fv_from_fid(4) == Vector3l(0, 7, 2));
    CHECK(child2.fv_from_fid(5) == Vector3l(3, 5, 8));
    CHECK(child2.fv_from_fid(6) == Vector3l(3, 8, 6));

    p_mul_manager.check_map_valid(parent);

    spdlog::info("===========================");
    spdlog::info("===========================");
    spdlog::info("===========================");
    // Do another edge_split
    {
        Tuple edge = parent.edge_tuple_between_v1_v2(0, 5, 4);
        operations::OperationSettings<operations::tri_mesh::EdgeSplit> settings;
        settings.initialize_invariants(parent);
        operations::tri_mesh::EdgeSplit split(parent, edge, settings);
        REQUIRE(split());
    }

    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0.is_connectivity_valid());
    REQUIRE(child1.is_connectivity_valid());
    REQUIRE(child2.is_connectivity_valid());

    CHECK(parent.fv_from_fid(2) == Vector3l(0, 2, 4));
    CHECK(parent.fv_from_fid(5) == Vector3l(5, 1, 2));
    CHECK(parent.fv_from_fid(3) == Vector3l(3, 1, 5));
    CHECK(parent.fv_from_fid(9) == Vector3l(0, 6, 2));
    CHECK(parent.fv_from_fid(10) == Vector3l(6, 5, 2));
    CHECK(parent.fv_from_fid(7) == Vector3l(3, 6, 0));
    CHECK(parent.fv_from_fid(8) == Vector3l(3, 5, 6));

    CHECK(child0.fv_from_fid(1) == Vector3l(3, 1, 2));
    CHECK(child0.fv_from_fid(3) == Vector3l(0, 4, 2));
    CHECK(child0.fv_from_fid(4) == Vector3l(4, 3, 2));

    CHECK(child1.fv_from_fid(4) == Vector3l(4, 1, 2));
    CHECK(child1.fv_from_fid(2) == Vector3l(3, 1, 4));
    CHECK(child1.fv_from_fid(8) == Vector3l(0, 5, 2));
    CHECK(child1.fv_from_fid(9) == Vector3l(5, 4, 2));
    CHECK(child1.fv_from_fid(6) == Vector3l(3, 5, 0));
    CHECK(child1.fv_from_fid(7) == Vector3l(3, 4, 5));

    CHECK(child2.fv_from_fid(2) == Vector3l(0, 2, 4));
    CHECK(child2.fv_from_fid(3) == Vector3l(7, 1, 2));
    CHECK(child2.fv_from_fid(5) == Vector3l(3, 5, 8));
    CHECK(child2.fv_from_fid(7) == Vector3l(0, 9, 2));
    CHECK(child2.fv_from_fid(8) == Vector3l(9, 7, 2));
    CHECK(child2.fv_from_fid(9) == Vector3l(3, 10, 6));
    CHECK(child2.fv_from_fid(10) == Vector3l(3, 8, 10));

    p_mul_manager.check_map_valid(parent);
}

TEST_CASE("test_collapse_multi_mesh", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(two_neighbors());
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::shared_ptr<DEBUG_TriMesh> child2_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());

    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;
    auto& child2 = *child2_ptr;

    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {0, 1, 2});
    auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0, 1});
    auto child2_map = multimesh::same_simplex_dimension_surjection(parent, child2, {0, 1, 2});

    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);
    parent.register_child_mesh(child2_ptr, child2_map);

    const auto& p_mul_manager = parent.multi_mesh_manager();

    p_mul_manager.check_map_valid(parent);

    {
        Tuple edge = parent.edge_tuple_between_v1_v2(1, 2, 0);
        operations::OperationSettings<operations::tri_mesh::EdgeCollapse> settings;
        settings.initialize_invariants(parent);
        operations::tri_mesh::EdgeCollapse collapse(parent, edge, settings);
        REQUIRE(collapse());
    }


    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0.is_connectivity_valid());
    REQUIRE(child1.is_connectivity_valid());
    REQUIRE(child2.is_connectivity_valid());
    p_mul_manager.check_map_valid(parent);

    CHECK(parent.fv_from_fid(1) == Vector3l(3, 2, 0));
    CHECK(parent.fv_from_fid(2) == Vector3l(0, 2, 4));
    CHECK(child0.fv_from_fid(1) == Vector3l(3, 2, 0));
    CHECK(child0.fv_from_fid(2) == Vector3l(0, 2, 4));
    CHECK(child1.fv_from_fid(1) == Vector3l(3, 2, 0));
    CHECK(child2.fv_from_fid(1) == Vector3l(3, 5, 6));
    CHECK(child2.fv_from_fid(2) == Vector3l(0, 2, 4));
}

