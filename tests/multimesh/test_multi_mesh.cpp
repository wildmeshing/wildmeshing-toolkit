#include <catch2/catch_test_macros.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/MapValidator.hpp>
#include <wmtk/multimesh/utils/check_map_valid.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "../tools/DEBUG_EdgeMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/EdgeMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::simplex;

using TM = TriMesh;


constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;


namespace {} // namespace

void print_tuple_map(const DEBUG_TriMesh& parent, const DEBUG_MultiMeshManager& p_mul_manager)
{
    int64_t child_id = 0;

    for (auto& child_data : p_mul_manager.children()) {
        std::cout << "child_id = " << child_id++ << std::endl;
        PrimitiveType map_ptype = child_data.mesh->top_simplex_type();
        auto parent_to_child_accessor = parent.create_const_accessor(child_data.map_handle);
        for (int64_t parent_gid = 0; parent_gid < parent.capacity(map_ptype); ++parent_gid) {
            auto parent_to_child_data = parent_to_child_accessor.const_vector_attribute(parent_gid);
            auto [parent_tuple, child_tuple] =
                wmtk::multimesh::utils::vectors_to_tuples(parent_to_child_data);
            std::cout << "parent gid = " << parent_gid << std::endl;
            std::cout << "parent_tuple = " << parent_tuple.as_string() << std::endl;
            std::cout << "child_tuple = " << child_tuple.as_string() << std::endl << std::endl;
        }
        std::cout << std::endl;
    }
}

TEST_CASE("test_register_child_mesh_bijection", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(two_neighbors());
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(two_neighbors());


    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;


    // check that bijection ~ surjection with the same number of elements
    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {0, 1, 2});
    auto child1_map = multimesh::same_simplex_dimension_bijection(parent, child1);

    // the maps should be the same
    REQUIRE(child0_map == child1_map);


    REQUIRE(wmtk::multimesh::utils::check_child_maps_valid(parent));
    REQUIRE(wmtk::multimesh::utils::check_child_maps_valid(child0));
    REQUIRE(wmtk::multimesh::utils::check_child_maps_valid(child1));
    REQUIRE(wmtk::multimesh::utils::MapValidator(parent).check_all());
    // some debug mode only checks
#if !defined(NDEBUG)
    // chekc that it fails when the # simplices is wrong
    std::shared_ptr<DEBUG_TriMesh> child2_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    CHECK_THROWS(multimesh::same_simplex_dimension_bijection(parent, *child2_ptr));

    // check that it fails when mesh dimensions are wrong
    std::shared_ptr<DEBUG_EdgeMesh> child3_ptr = std::make_shared<DEBUG_EdgeMesh>(single_line());
    CHECK_THROWS(multimesh::same_simplex_dimension_bijection(parent, *child3_ptr));
#endif
}

TEST_CASE("test_register_child_mesh", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());


    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;

    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {2});
    auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0, 1});

    // some debug mode only checks
#if !defined(NDEBUG)
    {
        // check for a dimension failure
        std::shared_ptr<DEBUG_EdgeMesh> child3_ptr =
            std::make_shared<DEBUG_EdgeMesh>(single_line());
        auto& child3 = *child3_ptr;
        CHECK_THROWS(multimesh::same_simplex_dimension_bijection(parent, child3));
    }
#endif

    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);

    REQUIRE(parent.get_child_meshes().size() == 2);
    const auto& p_mul_manager = parent.multi_mesh_manager();
    const auto& c0_mul_manager = child0.multi_mesh_manager();
    const auto& c1_mul_manager = child1.multi_mesh_manager();
    REQUIRE(p_mul_manager.get_child_meshes().size() == 2);
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
    REQUIRE(child0.absolute_multi_mesh_id() == std::vector<int64_t>{0});
    REQUIRE(child1.absolute_multi_mesh_id() == std::vector<int64_t>{1});

    auto check_mm_relationship = [](const auto& a, const auto& b) {
        CHECK(&a.get_multi_mesh_mesh(b.absolute_multi_mesh_id()) == &b);
        CHECK(&b.get_multi_mesh_mesh(a.absolute_multi_mesh_id()) == &a);
    };
    check_mm_relationship(parent, child0);
    check_mm_relationship(parent, child1);
    check_mm_relationship(child0, child1);

    // test attribute contents
    {
        const std::string c_to_p_name =
            DEBUG_MultiMeshManager::child_to_parent_map_attribute_name();
        const std::string p_to_c0_name =
            DEBUG_MultiMeshManager::parent_to_child_map_attribute_name(0);
        const std::string p_to_c1_name =
            DEBUG_MultiMeshManager::parent_to_child_map_attribute_name(1);
        REQUIRE(parent.has_attribute<int64_t>(p_to_c0_name, PF));
        REQUIRE(parent.has_attribute<int64_t>(p_to_c1_name, PF));
        REQUIRE(child0.has_attribute<int64_t>(c_to_p_name, PF));
        REQUIRE(child1.has_attribute<int64_t>(c_to_p_name, PF));

        auto parent_to_child0_handle =
            parent.get_attribute_handle<int64_t>(p_to_c0_name, PF).as<int64_t>();
        auto parent_to_child1_handle =
            parent.get_attribute_handle<int64_t>(p_to_c1_name, PF).as<int64_t>();
        auto child0_to_parent_handle =
            child0.get_attribute_handle<int64_t>(c_to_p_name, PF).as<int64_t>();
        auto child1_to_parent_handle =
            child1.get_attribute_handle<int64_t>(c_to_p_name, PF).as<int64_t>();

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


            for (int64_t parent_index = 0; parent_index < 3; ++parent_index) {
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


            for (int64_t parent_index = 0; parent_index < 3; ++parent_index) {
                auto ptuple = parent.tuple_from_id(PF, parent_index);
                Simplex psimplex = Simplex(parent, PF, ptuple);

                Tuple c0_expected = p_to_c0_map[parent_index];
                if (!c0_expected.is_null()) {
                    auto c0tuples = parent.map_to_child_tuples(child0, psimplex);
                    REQUIRE(c0tuples.size() == 1);
                    CHECK(c0tuples[0] == c0_expected);

                    auto c0tuples_lub = parent.lub_map_tuples(child0, psimplex);
                    CHECK(c0tuples == c0tuples_lub);
                }

                Tuple c1_expected = p_to_c1_map[parent_index];
                if (!c1_expected.is_null()) {
                    auto c1tuples = parent.map_to_child_tuples(child1, psimplex);
                    REQUIRE(c1tuples.size() == 1);
                    CHECK(c1tuples[0] == c1_expected);

                    auto c1tuples_lub = parent.lub_map_tuples(child1, psimplex);
                    CHECK(c1tuples == c1tuples_lub);
                }
            }
            for (size_t child0_index = 0; child0_index < c0_to_p_map.size(); ++child0_index) {
                auto tuple = child0.tuple_from_id(PF, child0_index);
                Simplex csimplex = Simplex(child0, PF, tuple);
                auto ptuple = child0.map_to_parent_tuple(csimplex);
                CHECK(ptuple == c0_to_p_map[child0_index]);

                auto parents_lub = child0.lub_map_tuples(parent, csimplex);
                REQUIRE(parents_lub.size() == 1);

                CHECK(parents_lub[0] == ptuple);
            }
            for (size_t child1_index = 0; child1_index < c1_to_p_map.size(); ++child1_index) {
                auto tuple = child1.tuple_from_id(PF, child1_index);
                Simplex csimplex = Simplex(child1, PF, tuple);
                auto ptuple = child1.map_to_parent_tuple(csimplex);
                CHECK(ptuple == c1_to_p_map[child1_index]);

                auto parents_lub = child1.lub_map_tuples(parent, csimplex);
                REQUIRE(parents_lub.size() == 1);
                CHECK(parents_lub[0] == ptuple);
            }
        }
    }

    // test actual api calls
    {
        // try the tuples that should succeed
        for (const auto& [ct, pt] : child0_map) {
            auto ncts =
                parent.map_to_child_tuples(child0, Simplex(parent, PrimitiveType::Triangle, pt));
            REQUIRE(ncts.size() == 1);
            auto nct = ncts[0];
            auto npt = child0.map_to_parent_tuple(Simplex(child0, PrimitiveType::Triangle, ct));

            CHECK(nct == ct);
            CHECK(npt == pt);
        }
        for (const auto& [ct, pt] : child1_map) {
            auto ncts =
                parent.map_to_child_tuples(child1, Simplex(parent, PrimitiveType::Triangle, pt));
            REQUIRE(ncts.size() == 1);
            auto nct = ncts[0];
            auto npt = child1.map_to_parent_tuple(Simplex(child1, PrimitiveType::Triangle, ct));

            CHECK(nct == ct);
            CHECK(npt == pt);
        }


        // go through simplex indices that aren't available in the map
        for (int64_t index = 0; index < 2; ++index) {
            auto pt = parent.tuple_from_id(PF, index);
            auto ncts = parent.map_to_child(child0, Simplex(parent, PrimitiveType::Triangle, pt));
            CHECK(ncts.size() == 0);
        }
        for (int64_t index = 2; index < 3; ++index) {
            auto pt = parent.tuple_from_id(PF, index);
            auto ncts = parent.map_to_child(child1, Simplex(parent, PrimitiveType::Triangle, pt));
            CHECK(ncts.size() == 0);
        }
    }


    p_mul_manager.check_map_valid(parent);
}

TEST_CASE("test_map_failures", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());


    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;

    Simplex s(parent, PrimitiveType::Vertex, parent.tuple_from_id(PrimitiveType::Vertex, 0));
    CHECK_THROWS(parent.map_to_parent_tuple(s));
    CHECK_THROWS(parent.map_to_child_tuples(child0, s));
    CHECK_THROWS(parent.map_to_parent(s));
    CHECK_THROWS(parent.map_to_child(child0, s));

    CHECK_THROWS(parent.map(child0, s));
    CHECK_THROWS(parent.map_tuples(child0, s));

    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {2});
    auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0, 1});

    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);
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
        auto tups = parent.map_to_child_tuples(mesh, Simplex(parent, PF, tuple));
        REQUIRE(tups.size() == 1);
        return tups[0];
    };

    Tuple edge = parent.edge_tuple_with_vs_and_t(1, 0, 0);
    Tuple edge_child0 = get_single_child_tuple(child0, edge);
    Tuple edge_child1 = get_single_child_tuple(child1, edge);
    Tuple edge_child2 = get_single_child_tuple(child2, edge);

    CHECK(edge_child0 == child0.edge_tuple_with_vs_and_t(1, 0, 0));
    CHECK(edge_child1 == child1.edge_tuple_with_vs_and_t(1, 0, 0));
    CHECK(edge_child2 == child2.edge_tuple_with_vs_and_t(1, 0, 0));

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

TEST_CASE("multi_mesh_register_2D_and_1D_single_triangle", "[multimesh][1D][2D]")
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
    REQUIRE(parent.get_child_meshes().size() == 2);
    REQUIRE(p_mul_manager.get_child_meshes().size() == 2);

    REQUIRE(p_mul_manager.is_root());
    REQUIRE(!c0_mul_manager.is_root());
    REQUIRE(!c1_mul_manager.is_root());

    // test id computation
    REQUIRE(parent.absolute_multi_mesh_id().empty());
    REQUIRE(child0.absolute_multi_mesh_id() == std::vector<int64_t>{0});
    REQUIRE(child1.absolute_multi_mesh_id() == std::vector<int64_t>{1});

    // test attribute contents
    {
        const std::string c_to_p_name =
            DEBUG_MultiMeshManager::child_to_parent_map_attribute_name();
        const std::string p_to_c0_name =
            DEBUG_MultiMeshManager::parent_to_child_map_attribute_name(0);
        const std::string p_to_c1_name =
            DEBUG_MultiMeshManager::parent_to_child_map_attribute_name(1);
        REQUIRE(parent.has_attribute<int64_t>(p_to_c0_name, PE));
        REQUIRE(parent.has_attribute<int64_t>(p_to_c1_name, PE));
        REQUIRE(child0.has_attribute<int64_t>(c_to_p_name, PE));
        REQUIRE(child1.has_attribute<int64_t>(c_to_p_name, PE));

        auto parent_to_child0_handle =
            parent.get_attribute_handle<int64_t>(p_to_c0_name, PE).as<int64_t>();
        auto parent_to_child1_handle =
            parent.get_attribute_handle<int64_t>(p_to_c1_name, PE).as<int64_t>();
        auto child0_to_parent_handle =
            child0.get_attribute_handle<int64_t>(c_to_p_name, PE).as<int64_t>();
        auto child1_to_parent_handle =
            child1.get_attribute_handle<int64_t>(c_to_p_name, PE).as<int64_t>();
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


            for (int64_t parent_index = 0; parent_index < 3; ++parent_index) {
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
        {
            std::vector<Tuple> p_to_c0_map{child0.tuple_from_id(PE, 0), Tuple(), Tuple()};

            std::vector<Tuple> p_to_c1_map{
                child1.tuple_from_id(PE, 0),
                Tuple(),
                child1.tuple_from_id(PE, 1)};

            std::vector<Tuple> c0_to_p_map{parent.tuple_from_id(PE, 0)};

            std::vector<Tuple> c1_to_p_map{
                parent.tuple_from_id(PE, 0),
                parent.tuple_from_id(PE, 2)};


            for (int64_t parent_index = 0; parent_index < 3; ++parent_index) {
                auto ptuple = parent.tuple_from_id(PE, parent_index);
                Simplex psimplex = Simplex(parent, PE, ptuple);

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
                auto tuple = child0.tuple_from_id(PE, child0_index);
                Simplex csimplex = Simplex(child0, PE, tuple);
                auto ptuple = child0.map_to_parent_tuple(csimplex);
                CHECK(ptuple == c0_to_p_map[child0_index]);
            }
            for (size_t child1_index = 0; child1_index < c1_to_p_map.size(); ++child1_index) {
                auto tuple = child1.tuple_from_id(PE, child1_index);
                Simplex csimplex = Simplex(child0, PE, tuple);
                auto ptuple = child1.map_to_parent_tuple(csimplex);
                CHECK(ptuple == c1_to_p_map[child1_index]);
            }
        }
    }

    // test actual api calls
    {
        // try the tuples that should succeed
        for (const auto& [ct, pt] : child0_map) {
            auto ncts = parent.map_to_child_tuples(child0, Simplex(parent, PE, pt));
            REQUIRE(ncts.size() == 1);
            auto nct = ncts[0];
            auto npt = child0.map_to_parent_tuple(Simplex(child0, PE, ct));

            CHECK(nct == ct);
            CHECK(npt == pt);
        }
        for (const auto& [ct, pt] : child1_map) {
            auto ncts = parent.map_to_child_tuples(child1, Simplex(parent, PE, pt));
            REQUIRE(ncts.size() == 1);
            auto nct = ncts[0];
            auto npt = child1.map_to_parent_tuple(Simplex(child1, PE, ct));

            CHECK(nct == ct);
            CHECK(npt == pt);
        }
        // go through simplex indices that aren't available in the map
        for (int64_t index = 1; index < 3; ++index) {
            auto pt = parent.tuple_from_id(PE, index);
            auto ncts = parent.map_to_child(child0, Simplex(parent, PE, pt));
            CHECK(ncts.size() == 0);
        }
        for (int64_t index = 1; index < 2; ++index) {
            auto pt = parent.tuple_from_id(PE, index);
            auto ncts = parent.map_to_child(child1, Simplex(parent, PE, pt));
            CHECK(ncts.size() == 0);
        }
    }

    p_mul_manager.check_map_valid(parent);
}

TEST_CASE("multi_mesh_register_between_2D_and_1D_one_ear", "[multimesh][1D][2D]")
{
    DEBUG_TriMesh parent = one_ear();
    std::shared_ptr<DEBUG_EdgeMesh> child0_ptr = std::make_shared<DEBUG_EdgeMesh>(single_line());
    std::shared_ptr<DEBUG_EdgeMesh> child1_ptr = std::make_shared<DEBUG_EdgeMesh>(two_segments());

    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;

    std::vector<std::array<Tuple, 2>> child0_map(1);
    std::vector<std::array<Tuple, 2>> child1_map(2);

    child0_map[0] = {child0.tuple_from_edge_id(0), parent.tuple_from_id(PE, 0)};
    child1_map[0] = {child1.tuple_from_edge_id(0), parent.tuple_from_id(PE, 0)};
    child1_map[1] = {child1.tuple_from_edge_id(1), parent.tuple_from_id(PE, 3)};

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
    REQUIRE(child0.absolute_multi_mesh_id() == std::vector<int64_t>{0});
    REQUIRE(child1.absolute_multi_mesh_id() == std::vector<int64_t>{1});

    // test attribute contents
    {
        const std::string c_to_p_name =
            DEBUG_MultiMeshManager::child_to_parent_map_attribute_name();
        const std::string p_to_c0_name =
            DEBUG_MultiMeshManager::parent_to_child_map_attribute_name(0);
        const std::string p_to_c1_name =
            DEBUG_MultiMeshManager::parent_to_child_map_attribute_name(1);
        REQUIRE(parent.has_attribute<int64_t>(p_to_c0_name, PE));
        REQUIRE(parent.has_attribute<int64_t>(p_to_c1_name, PE));
        REQUIRE(child0.has_attribute<int64_t>(c_to_p_name, PE));
        REQUIRE(child1.has_attribute<int64_t>(c_to_p_name, PE));

        auto parent_to_child0_handle =
            parent.get_attribute_handle<int64_t>(p_to_c0_name, PE).as<int64_t>();
        auto parent_to_child1_handle =
            parent.get_attribute_handle<int64_t>(p_to_c1_name, PE).as<int64_t>();
        auto child0_to_parent_handle =
            child0.get_attribute_handle<int64_t>(c_to_p_name, PE).as<int64_t>();
        auto child1_to_parent_handle =
            child1.get_attribute_handle<int64_t>(c_to_p_name, PE).as<int64_t>();
        auto parent_to_child0_acc = parent.create_const_accessor(parent_to_child0_handle);
        auto parent_to_child1_acc = parent.create_const_accessor(parent_to_child1_handle);
        auto child0_to_parent_acc = child0.create_const_accessor(child0_to_parent_handle);
        auto child1_to_parent_acc = child1.create_const_accessor(child1_to_parent_handle);

        // test read_tuple_map_attribute
        {
            std::vector<std::tuple<Tuple, Tuple>> p_to_c0_map{
                std::tuple<Tuple, Tuple>{parent.tuple_from_id(PE, 0), child0.tuple_from_id(PE, 0)},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()}};

            std::vector<std::tuple<Tuple, Tuple>> p_to_c1_map{
                std::tuple<Tuple, Tuple>{parent.tuple_from_id(PE, 0), child1.tuple_from_id(PE, 0)},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()},
                std::tuple<Tuple, Tuple>{parent.tuple_from_id(PE, 3), child1.tuple_from_id(PE, 1)},
                std::tuple<Tuple, Tuple>{Tuple(), Tuple()}};

            std::vector<std::tuple<Tuple, Tuple>> c0_to_p_map{
                std::tuple<Tuple, Tuple>{child0.tuple_from_id(PE, 0), parent.tuple_from_id(PE, 0)}};

            std::vector<std::tuple<Tuple, Tuple>> c1_to_p_map{
                std::tuple<Tuple, Tuple>{child1.tuple_from_id(PE, 0), parent.tuple_from_id(PE, 0)},
                std::tuple<Tuple, Tuple>{child1.tuple_from_id(PE, 1), parent.tuple_from_id(PE, 3)}};


            for (int64_t parent_index = 0; parent_index < 3; ++parent_index) {
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
        {
            std::vector<Tuple> p_to_c0_map{
                child0.tuple_from_id(PE, 0),
                Tuple(),
                Tuple(),
                Tuple(),
                Tuple()};

            std::vector<Tuple> p_to_c1_map{
                child1.tuple_from_id(PE, 0),
                Tuple(),
                Tuple(),
                child1.tuple_from_id(PE, 1),
                Tuple()};

            std::vector<Tuple> c0_to_p_map{parent.tuple_from_id(PE, 0)};

            std::vector<Tuple> c1_to_p_map{
                parent.tuple_from_id(PE, 0),
                parent.tuple_from_id(PE, 3)};


            for (int64_t parent_index = 0; parent_index < 3; ++parent_index) {
                auto ptuple = parent.tuple_from_id(PE, parent_index);
                Simplex psimplex = Simplex(parent, PE, ptuple);

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
                auto tuple = child0.tuple_from_id(PE, child0_index);
                Simplex csimplex = Simplex(child0, PE, tuple);
                auto ptuple = child0.map_to_parent_tuple(csimplex);
                CHECK(ptuple == c0_to_p_map[child0_index]);
            }
            for (size_t child1_index = 0; child1_index < c1_to_p_map.size(); ++child1_index) {
                auto tuple = child1.tuple_from_id(PE, child1_index);
                Simplex csimplex = Simplex(child1, PE, tuple);
                auto ptuple = child1.map_to_parent_tuple(csimplex);
                CHECK(ptuple == c1_to_p_map[child1_index]);
            }
        }
    }

    // test actual api calls
    {
        // try the tuples that should succeed
        for (const auto& [ct, pt] : child0_map) {
            auto ncts = parent.map_to_child_tuples(child0, Simplex(parent, PE, pt));
            REQUIRE(ncts.size() == 1);
            auto nct = ncts[0];
            auto npt = child0.map_to_parent_tuple(Simplex(child0, PE, ct));

            CHECK(nct == ct);
            CHECK(npt == pt);
        }
        for (const auto& [ct, pt] : child1_map) {
            auto ncts = parent.map_to_child_tuples(child1, Simplex(parent, PE, pt));
            REQUIRE(ncts.size() == 1);
            auto nct = ncts[0];
            auto npt = child1.map_to_parent_tuple(Simplex(child1, PE, ct));

            CHECK(nct == ct);
            CHECK(npt == pt);
        }
        // go through simplex indices that aren't available in the map
        for (int64_t index = 1; index < 5; ++index) {
            auto pt = parent.tuple_from_id(PE, index);
            auto ncts = parent.map_to_child(child0, Simplex(parent, PE, pt));
            CHECK(ncts.size() == 0);
        }
        for (int64_t index = 1; index < 3; ++index) {
            auto pt = parent.tuple_from_id(PE, index);
            auto ncts = parent.map_to_child(child1, Simplex(parent, PE, pt));
            CHECK(ncts.size() == 0);
        }
        for (int64_t index = 4; index < 5; ++index) {
            auto pt = parent.tuple_from_id(PE, index);
            auto ncts = parent.map_to_child(child1, Simplex(parent, PE, pt));
            CHECK(ncts.size() == 0);
        }
    }

    p_mul_manager.check_map_valid(parent);
}

TEST_CASE("test_split_multi_mesh_1D_2D", "[multimesh][1D][2D]")
{
    DEBUG_TriMesh parent = one_ear();
    std::shared_ptr<DEBUG_EdgeMesh> child0_ptr = std::make_shared<DEBUG_EdgeMesh>(single_line());
    std::shared_ptr<DEBUG_EdgeMesh> child1_ptr = std::make_shared<DEBUG_EdgeMesh>(two_segments());

    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;

    std::vector<std::array<Tuple, 2>> child0_map(1);
    std::vector<std::array<Tuple, 2>> child1_map(2);

    child0_map[0] = {child0.tuple_from_edge_id(0), parent.tuple_from_id(PE, 0)};
    child1_map[0] = {child1.tuple_from_edge_id(0), parent.tuple_from_id(PE, 0)};
    child1_map[1] = {child1.tuple_from_edge_id(1), parent.tuple_from_id(PE, 3)};

    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);

    const auto& p_mul_manager = parent.multi_mesh_manager();
    // const auto& c0_mul_manager = child0.multi_mesh_manager();
    // const auto& c1_mul_manager = child1.multi_mesh_manager();

    {
        Tuple edge = parent.edge_tuple_with_vs_and_t(0, 1, 0);
        operations::EdgeSplit op(parent);
        REQUIRE(!op(Simplex::edge(parent, edge)).empty());
    }

    logger().debug("parent.capacity(PF) = {}", parent.capacity(PF));
    logger().debug("child0.capacity(PE) = {}", child0.capacity(PE));
    logger().debug("child1.capacity(PE) = {}", child1.capacity(PE));
    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0.is_connectivity_valid());
    REQUIRE(child1.is_connectivity_valid());
    p_mul_manager.check_map_valid(parent);

    print_tuple_map(parent, p_mul_manager);

    // Do another edge_split
    {
        Tuple edge = parent.edge_tuple_with_vs_and_t(1, 2, 3);
#if defined(WMTK_ENABLE_HASH_UPDATE)
        REQUIRE(parent.is_valid_with_hash(edge));
#else
        REQUIRE(parent.is_valid(edge));
#endif
        operations::EdgeSplit op(parent);
        REQUIRE(!op(Simplex::edge(parent, edge)).empty());
    }
    logger().debug("parent.capacity(PF) = {}", parent.capacity(PF));
    logger().debug("child0.capacity(PE) = {}", child0.capacity(PE));
    logger().debug("child1.capacity(PE) = {}", child1.capacity(PE));
    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0.is_connectivity_valid());
    REQUIRE(child1.is_connectivity_valid());
    p_mul_manager.check_map_valid(parent);

    print_tuple_map(parent, p_mul_manager);
}

TEST_CASE("test_collapse_multi_mesh_1D_2D", "[multimesh][1D][2D]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());
    std::shared_ptr<DEBUG_EdgeMesh> child1_ptr = std::make_shared<DEBUG_EdgeMesh>(two_segments());
    std::shared_ptr<DEBUG_EdgeMesh> child2_ptr = std::make_shared<DEBUG_EdgeMesh>(single_line());


    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;
    auto& child2 = *child2_ptr;

    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {0, 1, 2});

    std::vector<std::array<Tuple, 2>> child1_map(2);
    child1_map[0] = {child1.tuple_from_edge_id(0), parent.edge_tuple_with_vs_and_t(0, 1, 0)};
    child1_map[1] = {child1.tuple_from_edge_id(1), parent.edge_tuple_with_vs_and_t(1, 2, 0)};
    std::vector<std::array<Tuple, 2>> child2_map(1);
    child2_map[0] = {child2.tuple_from_edge_id(0), parent.edge_tuple_with_vs_and_t(0, 4, 2)};

    // parent.register_child_mesh(child0_ptr, child0_map);
    // parent.register_child_mesh(child1_ptr, child1_map);
    parent.register_child_mesh(child2_ptr, child2_map);

    const auto& p_mul_manager = parent.multi_mesh_manager();

    p_mul_manager.check_map_valid(parent);
    print_tuple_map(parent, p_mul_manager);

    SECTION("collapse case 1")
    {
        {
            Tuple edge = parent.edge_tuple_with_vs_and_t(1, 2, 0);
            operations::EdgeCollapse collapse(parent);
            collapse.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(parent));

            REQUIRE(!collapse(Simplex::edge(parent, edge)).empty());
        }
        print_tuple_map(parent, p_mul_manager);

        p_mul_manager.check_map_valid(parent);
    }

    SECTION("collapse case 2")
    {
        {
            Tuple edge = parent.edge_tuple_with_vs_and_t(2, 4, 2);
            operations::EdgeCollapse collapse(parent);
            collapse.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(parent));
            REQUIRE(!collapse(Simplex::edge(parent, edge)).empty());
        }

        p_mul_manager.check_map_valid(parent);
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
    REQUIRE(child0.absolute_multi_mesh_id() == std::vector<int64_t>{0});
    REQUIRE(child1.absolute_multi_mesh_id() == std::vector<int64_t>{1});
    REQUIRE(child2.absolute_multi_mesh_id() == std::vector<int64_t>{2});

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
        Tuple edge = parent.edge_tuple_with_vs_and_t(1, 0, 1);
        simplex::Simplex edge_simplex = simplex::Simplex(parent, PrimitiveType::Edge, edge);

        Tuple edge_f0 = parent.edge_tuple_with_vs_and_t(1, 0, 0);
        simplex::Simplex edge_f0_simplex = simplex::Simplex(parent, PrimitiveType::Edge, edge_f0);

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
#if defined(WMTK_ENABLE_HASH_UPDATE)
            REQUIRE(child0.is_valid_with_hash(cs.tuple()));
#else
            REQUIRE(child0.is_valid(cs.tuple()));
#endif
            REQUIRE(wmtk::simplex::utils::SimplexComparisons::equal(child0, cs, edge_f0_simplex));
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
#if defined(WMTK_ENABLE_HASH_UPDATE)
            REQUIRE(child1.is_valid_with_hash(cs.tuple()));
#else
            REQUIRE(child1.is_valid(cs.tuple()));
#endif
            REQUIRE(wmtk::simplex::utils::SimplexComparisons::equal(child1, cs, edge_simplex));
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

            std::cout << cs0.tuple().as_string() << " " << cs1.tuple().as_string() << std::endl;
            std::cout << edge_f0_simplex.tuple().as_string() << " "
                      << edge_simplex.tuple().as_string() << std::endl;

#if defined(WMTK_ENABLE_HASH_UPDATE)
            REQUIRE(child2.is_valid_with_hash(cs0.tuple()));
            REQUIRE(child2.is_valid_with_hash(cs1.tuple()));
#else
            REQUIRE(child2.is_valid(cs0.tuple()));
            REQUIRE(child2.is_valid(cs1.tuple()));
#endif
            REQUIRE(wmtk::simplex::utils::SimplexComparisons::equal(child2, cs0, edge_f0_simplex));
            REQUIRE(cs1.tuple() == edge_simplex.tuple());
            REQUIRE(cs1.primitive_type() == edge_simplex.primitive_type());
        }

        operations::EdgeSplit split(parent);
        REQUIRE(!split(Simplex::edge(parent, edge)).empty());
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

    // Do another edge_split
    {
        Tuple edge = parent.edge_tuple_with_vs_and_t(0, 5, 4);
        operations::EdgeSplit split(parent);
        REQUIRE(!split(Simplex::edge(parent, edge)).empty());
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
        Tuple edge = parent.edge_tuple_with_vs_and_t(1, 2, 0);
        operations::EdgeCollapse collapse(parent);
        collapse.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(parent));
        REQUIRE(!collapse(Simplex::edge(parent, edge)).empty());
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

TEST_CASE("test_multimesh_link_cond", "[multimesh][2D]")
{
    DEBUG_TriMesh parent = two_neighbors_plus_one();

    std::shared_ptr<DEBUG_TriMesh> tri_child0_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors());
    std::shared_ptr<DEBUG_TriMesh> tri_child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
    std::shared_ptr<DEBUG_TriMesh> tri_child2_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());
    auto& tri_child0 = *tri_child0_ptr;
    auto& tri_child1 = *tri_child1_ptr;
    auto& tri_child2 = *tri_child2_ptr;
    auto tri_child0_map =
        multimesh::same_simplex_dimension_surjection(parent, tri_child0, {0, 1, 2});
    auto tri_child1_map = multimesh::same_simplex_dimension_surjection(parent, tri_child1, {0, 1});
    auto tri_child2_map =
        multimesh::same_simplex_dimension_surjection(parent, tri_child2, {0, 1, 2});

    std::shared_ptr<DEBUG_EdgeMesh> edge_child0_ptr =
        std::make_shared<DEBUG_EdgeMesh>(single_line());
    std::shared_ptr<DEBUG_EdgeMesh> edge_child1_ptr =
        std::make_shared<DEBUG_EdgeMesh>(two_segments());
    auto& edge_child0 = *edge_child0_ptr;
    auto& edge_child1 = *edge_child1_ptr;

    std::vector<std::array<Tuple, 2>> edge_child0_map(1);
    std::vector<std::array<Tuple, 2>> edge_child1_map(2);
    edge_child0_map[0] = {
        edge_child0.tuple_from_edge_id(0),
        parent.edge_tuple_with_vs_and_t(0, 1, 0)};
    edge_child1_map[0] = {
        edge_child1.tuple_from_edge_id(0),
        parent.edge_tuple_with_vs_and_t(0, 1, 0)};
    edge_child1_map[1] = {
        edge_child1.tuple_from_edge_id(1),
        parent.edge_tuple_with_vs_and_t(1, 2, 0)};


    const auto& p_mul_manager = parent.multi_mesh_manager();


    SECTION("Case 1 should succeed")
    {
        parent.register_child_mesh(tri_child0_ptr, tri_child0_map);
        parent.register_child_mesh(tri_child1_ptr, tri_child1_map);
        parent.register_child_mesh(tri_child2_ptr, tri_child2_map);
        p_mul_manager.check_map_valid(parent);
        {
            Tuple edge = parent.edge_tuple_with_vs_and_t(1, 2, 0);
            operations::EdgeCollapse collapse(parent);
            collapse.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(parent));
            REQUIRE(!collapse(Simplex::edge(parent, edge)).empty());
        }


        REQUIRE(parent.is_connectivity_valid());
        REQUIRE(tri_child0.is_connectivity_valid());
        REQUIRE(tri_child1.is_connectivity_valid());
        REQUIRE(tri_child2.is_connectivity_valid());
        p_mul_manager.check_map_valid(parent);
    }

    SECTION("Case 2 should fail")
    {
        parent.register_child_mesh(tri_child0_ptr, tri_child0_map);
        parent.register_child_mesh(tri_child1_ptr, tri_child1_map);
        parent.register_child_mesh(tri_child2_ptr, tri_child2_map);
        p_mul_manager.check_map_valid(parent);
        {
            Tuple edge = parent.edge_tuple_with_vs_and_t(0, 2, 0);
            operations::EdgeCollapse collapse(parent);
            collapse.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(parent));
            bool is_collapse_fail = collapse(Simplex::edge(parent, edge)).empty();
            REQUIRE(is_collapse_fail);
        }

        REQUIRE(parent.is_connectivity_valid());
        REQUIRE(tri_child0.is_connectivity_valid());
        REQUIRE(tri_child1.is_connectivity_valid());
        REQUIRE(tri_child2.is_connectivity_valid());
        p_mul_manager.check_map_valid(parent);
    }
}


TEST_CASE("test_split_multi_mesh_1D_2D_single_triangle", "[multimesh][1D][2D]")
{
    DEBUG_TriMesh parent = single_triangle();
    std::shared_ptr<DEBUG_EdgeMesh> child0_ptr = std::make_shared<DEBUG_EdgeMesh>(single_line());

    auto& child0 = *child0_ptr;

    std::vector<std::array<Tuple, 2>> child0_map(1);
    std::vector<std::array<Tuple, 2>> child1_map(2);

    child0_map[0] = {child0.tuple_from_edge_id(0), parent.tuple_from_id(PE, 0)};


    parent.register_child_mesh(child0_ptr, child0_map);

    const auto& p_mul_manager = parent.multi_mesh_manager();

    {
        Tuple edge = parent.edge_tuple_with_vs_and_t(0, 2, 0);
        operations::EdgeSplit split(parent);
        REQUIRE(!split(Simplex::edge(parent, edge)).empty());
    }

    logger().debug("parent.capacity(PF) = {}", parent.capacity(PF));
    logger().debug("child0.capacity(PE) = {}", child0.capacity(PE));
    REQUIRE(parent.is_connectivity_valid());
    REQUIRE(child0.is_connectivity_valid());
    p_mul_manager.check_map_valid(parent);

    print_tuple_map(parent, p_mul_manager);
}

TEST_CASE("test_deregister_child_mesh", "[multimesh]")
{
    DEBUG_TriMesh parent = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> child0_ptr = std::make_shared<DEBUG_TriMesh>(single_triangle());
    std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());


    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;

    {
        auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {2});
        auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0, 1});

        parent.register_child_mesh(child0_ptr, child0_map);
        parent.register_child_mesh(child1_ptr, child1_map);
    }

    REQUIRE(parent.get_child_meshes().size() == 2);
    const auto& p_mul_manager = parent.multi_mesh_manager();
    const auto& c0_mul_manager = child0.multi_mesh_manager();
    const auto& c1_mul_manager = child1.multi_mesh_manager();
    REQUIRE(p_mul_manager.get_child_meshes().size() == 2);
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

    SECTION("remove_0")
    {
        parent.deregister_child_mesh(child0_ptr);
        CHECK(parent.get_child_meshes().size() == 1);
        CHECK(p_mul_manager.get_child_meshes().size() == 1);
        CHECK(p_mul_manager.children().size() == 1);
        CHECK(p_mul_manager.children()[0].mesh == child1_ptr);
        CHECK(c0_mul_manager.children().size() == 0);
        CHECK(c1_mul_manager.children().size() == 0);
        CHECK(c0_mul_manager.get_root_mesh(child0) == *child0_ptr);
        CHECK(c1_mul_manager.get_root_mesh(child1) == parent);

        CHECK(p_mul_manager.is_root());
        CHECK(c0_mul_manager.is_root());
        CHECK_FALSE(c1_mul_manager.is_root());
    }
    SECTION("remove_1")
    {
        parent.deregister_child_mesh(child1_ptr);
        CHECK(parent.get_child_meshes().size() == 1);
        CHECK(p_mul_manager.get_child_meshes().size() == 1);
        CHECK(p_mul_manager.children().size() == 1);
        CHECK(p_mul_manager.children()[0].mesh == child0_ptr);
        CHECK(c0_mul_manager.children().size() == 0);
        CHECK(c1_mul_manager.children().size() == 0);
        CHECK(c0_mul_manager.get_root_mesh(child0) == parent);
        CHECK(c1_mul_manager.get_root_mesh(child1) == *child1_ptr);

        CHECK(p_mul_manager.is_root());
        CHECK(c1_mul_manager.is_root());
        CHECK_FALSE(c0_mul_manager.is_root());
    }
}
