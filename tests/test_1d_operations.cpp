#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"


using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::simplex;
using namespace wmtk::operations;

using EM = EdgeMesh;
using MapResult = typename Eigen::Matrix<int64_t, Eigen::Dynamic, 1>::MapType;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;

TEST_CASE("simplices_to_delete_for_split_1D", "[operations][1D]")
{
    SECTION("single line")
    {
        DEBUG_EdgeMesh m = single_line();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        REQUIRE(m.is_valid(edge));


        auto executor = m.get_emoe(edge);

        executor.split_edge();
        REQUIRE(m.is_connectivity_valid());
        const auto& ids_to_delete = executor.simplex_ids_to_delete;
        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);
    }

    SECTION("self loop")
    {
        DEBUG_EdgeMesh m = self_loop();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        REQUIRE(m.is_valid(edge));

        auto executor = m.get_emoe(edge);

        executor.split_edge();
        REQUIRE(m.is_connectivity_valid());
        const auto& ids_to_delete = executor.simplex_ids_to_delete;
        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);
    }
}

TEST_CASE("simplices_to_delete_for_collapse_1D", "[operations][1D]")
{
    SECTION("multiple_lines")
    {
        DEBUG_EdgeMesh m = multiple_lines();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 2;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        REQUIRE(m.is_valid(edge));
        auto executor = m.get_emoe(edge);

        executor.collapse_edge();
        // REQUIRE(m.is_connectivity_valid());
        const auto& ids_to_delete = executor.simplex_ids_to_delete;
        REQUIRE(ids_to_delete[0].size() == 1);
        REQUIRE(ids_to_delete[0][0] == 2);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);
    }

    SECTION("two_line_loop")
    {
        DEBUG_EdgeMesh m = two_line_loop();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        REQUIRE(m.is_valid(edge));
        auto executor = m.get_emoe(edge);

        executor.collapse_edge();
        const auto& ids_to_delete = executor.simplex_ids_to_delete;
        REQUIRE(ids_to_delete[0].size() == 1);
        REQUIRE(ids_to_delete[0][0] == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);
    }
}


TEST_CASE("collapse_edge_1D", "[operations][1D]")
{
    SECTION("multiple_lines")
    {
        DEBUG_EdgeMesh m = multiple_lines();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 2;
        Tuple edge = m.tuple_from_edge_id(edge_id);

        REQUIRE(m.is_valid(edge));
        const int64_t vertex_id = m.id(edge, PV);

        EdgeCollapse collapse(m);
        auto res = collapse(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret_tuple = res.front().tuple();

        CHECK(m.is_connectivity_valid());
        CHECK(!ret_tuple.is_null()); // collapse operation is valid
        // check return tuple
        CHECK(m.id(ret_tuple, PE) == 1);
        CHECK(m.id(ret_tuple, PV) == 3);
        // check delete
        CHECK(m.is_simplex_deleted(PE, edge_id));
        CHECK(m.is_simplex_deleted(PV, vertex_id));

        auto& ve = m.create_base_accessor<int64_t>(m.ve_handle());
        auto& ee = m.create_base_accessor<int64_t>(m.e_handle(PE));
        auto& ev = m.create_base_accessor<int64_t>(m.e_handle(PV));
        // check ve, ee, ev
        CHECK(ve.scalar_attribute(3) == 3);
        CHECK(ee.vector_attribute(1)[1] == 3);
        CHECK(ee.vector_attribute(3)[0] == 1);
        CHECK(ev.vector_attribute(1)[1] == 3);
        CHECK(ev.vector_attribute(3)[0] == 3);
    }

    SECTION("single_line")
    {
        DEBUG_EdgeMesh m = single_line();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        REQUIRE(m.is_valid(edge));


        EdgeCollapse collapse(m);
        collapse.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        auto res = collapse(Simplex::edge(m, edge));
        REQUIRE(res.empty());
        CHECK(m.is_connectivity_valid());
    }

    SECTION("self_loop")
    {
        DEBUG_EdgeMesh m = self_loop();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        REQUIRE(m.is_valid(edge));


        EdgeCollapse collapse(m);
        collapse.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        auto res = collapse(Simplex::edge(m, edge));
        REQUIRE(res.empty());

        CHECK(m.is_connectivity_valid());
    }

    SECTION("two_line_loop")
    {
        DEBUG_EdgeMesh m = two_line_loop();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        const int64_t vertex_id = m.id(edge, PV);
        REQUIRE(m.is_valid(edge));

        EdgeCollapse collapse(m);
        auto res = collapse(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret_tuple = res.front().tuple();

        CHECK(m.is_connectivity_valid());
        CHECK(!ret_tuple.is_null()); // collapse operation is valid
        // check return tuple
        CHECK(m.id(ret_tuple, PE) == 1);
        CHECK(m.id(ret_tuple, PV) == 1);
        // check delete
        CHECK(m.is_simplex_deleted(PE, edge_id));
        CHECK(m.is_simplex_deleted(PV, vertex_id));

        auto &ve = m.create_base_accessor<int64_t>(m.ve_handle());
        auto &ee = m.create_base_accessor<int64_t>(m.e_handle(PE));
        auto &ev = m.create_base_accessor<int64_t>(m.e_handle(PV));
        // check ve, ee, ev
        CHECK(ve.scalar_attribute(1) == 1);
        CHECK(ee.vector_attribute(1)[0] == 1);
        CHECK(ee.vector_attribute(1)[1] == 1);
        CHECK(ev.vector_attribute(1)[0] == 1);
        CHECK(ev.vector_attribute(1)[1] == 1);
    }

    SECTION("loop_lines")
    {
        DEBUG_EdgeMesh m = loop_lines();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        const int64_t vertex_id = m.id(edge, PV);

        REQUIRE(m.is_valid(edge));

        EdgeCollapse collapse(m);
        auto res = collapse(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret_tuple = res.front().tuple();

        CHECK(m.is_connectivity_valid());
        CHECK(!ret_tuple.is_null()); // collapse operation is valid
        // check return tuple
        CHECK(m.id(ret_tuple, PE) == 5);
        CHECK(m.id(ret_tuple, PV) == 1);
        // check delete
        CHECK(m.is_simplex_deleted(PE, edge_id));
        CHECK(m.is_simplex_deleted(PV, vertex_id));


        auto &ve = m.create_base_accessor<int64_t>(m.ve_handle());
        auto &ee = m.create_base_accessor<int64_t>(m.e_handle(PE));
        auto &ev = m.create_base_accessor<int64_t>(m.e_handle(PV));
        // check ve, ee, ev
        CHECK(ve.scalar_attribute(1) == 1);
        CHECK(ee.vector_attribute(1)[0] == 5);
        CHECK(ee.vector_attribute(5)[1] == 1);
        CHECK(ev.vector_attribute(1)[0] == 1);
        CHECK(ev.vector_attribute(5)[1] == 1);
    }
}

TEST_CASE("split_edge_1D", "[operations][1D]")
{
    SECTION("multiple_lines")
    {
        DEBUG_EdgeMesh m = multiple_lines();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 2;
        Tuple edge = m.tuple_from_edge_id(edge_id);

        REQUIRE(m.is_valid(edge));
        const int64_t vertex_id = m.id(edge, PV);

        EdgeSplit split(m);
        auto res = split(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret_tuple = res.front().tuple();

        CHECK(m.is_connectivity_valid());
        CHECK(!ret_tuple.is_null()); // split operation is valid
        // check return tuple
        CHECK(m.id(ret_tuple, PE) == 6);
        CHECK(m.id(ret_tuple, PV) == 6);
        // check delete
        CHECK(m.is_simplex_deleted(PE, edge_id));

        auto &ve = m.create_base_accessor<int64_t>(m.ve_handle());
        auto &ee = m.create_base_accessor<int64_t>(m.e_handle(PE));
        auto &ev = m.create_base_accessor<int64_t>(m.e_handle(PV));
        // check ve, ee, ev
        CHECK(ve.scalar_attribute(3) == 6);
        CHECK(ve.scalar_attribute(6) == 5);
        CHECK(ve.scalar_attribute(2) == 5);

        CHECK(ee.vector_attribute(1)[1] == 5);
        CHECK(ee.vector_attribute(5)[0] == 1);
        CHECK(ee.vector_attribute(5)[1] == 6);
        CHECK(ee.vector_attribute(6)[0] == 5);
        CHECK(ee.vector_attribute(6)[1] == 3);
        CHECK(ee.vector_attribute(3)[0] == 6);

        CHECK(ev.vector_attribute(5)[0] == 2);
        CHECK(ev.vector_attribute(5)[1] == 6);
        CHECK(ev.vector_attribute(6)[0] == 6);
        CHECK(ev.vector_attribute(6)[1] == 3);
    }

    SECTION("single_line")
    {
        DEBUG_EdgeMesh m = single_line();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        const int64_t vertex_id = m.id(edge, PV);
        REQUIRE(m.is_valid(edge));

        EdgeSplit split(m);
        auto res = split(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret_tuple = res.front().tuple();

        CHECK(m.is_connectivity_valid());
        CHECK(!ret_tuple.is_null()); // split opearation is valid
        // check return tuple
        CHECK(m.id(ret_tuple, PE) == 2);
        CHECK(m.id(ret_tuple, PV) == 2);
        // check delete
        CHECK(m.is_simplex_deleted(PE, edge_id));

        auto &ve = m.create_base_accessor<int64_t>(m.ve_handle());
        auto &ee = m.create_base_accessor<int64_t>(m.e_handle(PE));
        auto &ev = m.create_base_accessor<int64_t>(m.e_handle(PV));
        // check ve, ee, ev
        CHECK(ve.scalar_attribute(0) == 1);
        CHECK(ve.scalar_attribute(2) == 1);
        CHECK(ve.scalar_attribute(1) == 2);

        CHECK(ee.vector_attribute(1)[0] == -1);
        CHECK(ee.vector_attribute(1)[1] == 2);
        CHECK(ee.vector_attribute(2)[0] == 1);
        CHECK(ee.vector_attribute(2)[1] == -1);

        CHECK(ev.vector_attribute(1)[0] == 0);
        CHECK(ev.vector_attribute(1)[1] == 2);
        CHECK(ev.vector_attribute(2)[0] == 2);
        CHECK(ev.vector_attribute(2)[1] == 1);
    }

    SECTION("self_loop")
    {
        DEBUG_EdgeMesh m = self_loop();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        const int64_t vertex_id = m.id(edge, PV);
        REQUIRE(m.is_valid(edge));

        EdgeSplit split(m);
        auto res = split(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret_tuple = res.front().tuple();

        CHECK(m.is_connectivity_valid());
        CHECK(!ret_tuple.is_null()); // split opearation is valid
        // check return tuple
        CHECK(m.id(ret_tuple, PE) == 2);
        CHECK(m.id(ret_tuple, PV) == 1);
        // check delete
        CHECK(m.is_simplex_deleted(PE, edge_id));

        auto &ve = m.create_base_accessor<int64_t>(m.ve_handle());
        auto &ee = m.create_base_accessor<int64_t>(m.e_handle(PE));
        auto &ev = m.create_base_accessor<int64_t>(m.e_handle(PV));
        // check ve, ee, ev
        CHECK(ve.scalar_attribute(0) == 2);
        CHECK(ve.scalar_attribute(1) == 1);

        CHECK(ee.vector_attribute(1)[0] == 2);
        CHECK(ee.vector_attribute(1)[1] == 2);
        CHECK(ee.vector_attribute(2)[0] == 1);
        CHECK(ee.vector_attribute(2)[1] == 1);

        CHECK(ev.vector_attribute(1)[0] == 0);
        CHECK(ev.vector_attribute(1)[1] == 1);
        CHECK(ev.vector_attribute(2)[0] == 1);
        CHECK(ev.vector_attribute(2)[1] == 0);
    }

    SECTION("two_line_loop")
    {
        DEBUG_EdgeMesh m = two_line_loop();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        const int64_t vertex_id = m.id(edge, PV);
        REQUIRE(m.is_valid(edge));

        EdgeSplit split(m);
        auto res = split(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret_tuple = res.front().tuple();

        CHECK(m.is_connectivity_valid());
        CHECK(!ret_tuple.is_null()); // split operation is valid
        // check return tuple
        CHECK(m.id(ret_tuple, PE) == 3);
        CHECK(m.id(ret_tuple, PV) == 2);
        // check delete
        CHECK(m.is_simplex_deleted(PE, edge_id));

        auto &ve = m.create_base_accessor<int64_t>(m.ve_handle());
        auto &ee = m.create_base_accessor<int64_t>(m.e_handle(PE));
        auto &ev = m.create_base_accessor<int64_t>(m.e_handle(PV));
        // check ve, ee, ev
        CHECK(ve.scalar_attribute(0) == 2);
        CHECK(ve.scalar_attribute(1) == 3);
        CHECK(ve.scalar_attribute(2) == 2);

        CHECK(ee.vector_attribute(1)[1] == 2);
        CHECK(ee.vector_attribute(2)[0] == 1);
        CHECK(ee.vector_attribute(2)[1] == 3);
        CHECK(ee.vector_attribute(3)[0] == 2);
        CHECK(ee.vector_attribute(3)[1] == 1);
        CHECK(ee.vector_attribute(1)[0] == 3);

        CHECK(ev.vector_attribute(2)[0] == 0);
        CHECK(ev.vector_attribute(2)[1] == 2);
        CHECK(ev.vector_attribute(3)[0] == 2);
        CHECK(ev.vector_attribute(3)[1] == 1);
    }

    SECTION("loop_lines")
    {
        DEBUG_EdgeMesh m = loop_lines();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        const int64_t vertex_id = m.id(edge, PV);
        REQUIRE(m.is_valid(edge));

        EdgeSplit split(m);
        auto res = split(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret_tuple = res.front().tuple();

        CHECK(m.is_connectivity_valid());
        CHECK(!ret_tuple.is_null()); // split operation is valid
        // check return tuple
        CHECK(m.id(ret_tuple, PE) == 7);
        CHECK(m.id(ret_tuple, PV) == 6);
        // check delete
        CHECK(m.is_simplex_deleted(PE, edge_id));

        auto& ve = m.create_base_accessor<int64_t>(m.ve_handle());
        auto& ee = m.create_base_accessor<int64_t>(m.e_handle(PE));
        auto& ev = m.create_base_accessor<int64_t>(m.e_handle(PV));
        // check ve, ee, ev
        CHECK(ve.scalar_attribute(0) == 6);
        CHECK(ve.scalar_attribute(1) == 7);
        CHECK(ve.scalar_attribute(6) == 6);

        CHECK(ee.vector_attribute(5)[1] == 6);
        CHECK(ee.vector_attribute(6)[0] == 5);
        CHECK(ee.vector_attribute(6)[1] == 7);
        CHECK(ee.vector_attribute(7)[0] == 6);
        CHECK(ee.vector_attribute(7)[1] == 1);
        CHECK(ee.vector_attribute(1)[0] == 7);

        CHECK(ev.vector_attribute(6)[0] == 0);
        CHECK(ev.vector_attribute(6)[1] == 6);
        CHECK(ev.vector_attribute(7)[0] == 6);
        CHECK(ev.vector_attribute(7)[1] == 1);
    }
}
