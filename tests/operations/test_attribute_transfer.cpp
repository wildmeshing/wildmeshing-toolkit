#include <catch2/catch_test_macros.hpp>
#include <wmtk/operations/AttributeTransferStrategy.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
TEST_CASE("split_edge", "[operations][split][2D]")
{
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /
    //    7---8
    DEBUG_TriMesh m = hex_plus_two_with_position();


    // this handel already has default behaviors so lets leave it alone
    auto pos_handle = m.get_attribute_handle<double>("vertices");
    //
    //


    auto edge_length_handle = m.register_attribute("edge_length", PrimitiveType::Edge, 1);

    auto compute_edge_length = [](const auto& P) {
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };

    std::shared_ptr el_behavior =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy>(
            edge_length_handle,
            pos_handle,
            compute_edge_length);


    auto edges = m.get_all(PrimitiveType::Edge);
    auto pos_acc = pos.create_const_accessor();
    auto el_acc = edge_length_handle.create_const_accessor();

    auto check_lengths = [&]() {
        for (const auto& e : edges) {
            el_behavior->update(e);


            auto v0 = e;
            auto v1 = m.switch_vertex(e);

            auto pos0 = pos_acc.vector_attribute(v0);
            auto pos1 = pos_acc.vector_attribute(v1);

            double len = (pos0 - pos1).norm();

            double len2 = el_acc.scalar_attribute(e);

            CHECK(len == len2);
        }
    };


    check_lengths();

    // add strategy (api can be cleaned up of course)
    m.m_transfer_strategies.emplace_back(el_behavior);

    Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
    m.split_edge(edge, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
    check_lengths();

    Tuple edge2 = m.edge_tuple_between_v1_v2(3, 0, 0);
    m.split_edge(edge2, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
    check_lengths();

    Tuple edge3 = m.edge_tuple_between_v1_v2(4, 7, 6);
    REQUIRE(m.is_valid_slow(edge3));
    m.split_edge(edge3, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
    check_lengths();

    Tuple edge4 = m.edge_tuple_between_v1_v2(4, 9, 8);
    m.split_edge(edge4, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
    check_lengths();

    Tuple edge5 = m.edge_tuple_between_v1_v2(5, 6, 4);
    m.split_edge(edge5, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
    check_lengths();
}
TEST_CASE("collapse_edge", "[operations][collapse][2D]")
{
    using namespace operations;

    DEBUG_TriMesh m = hex_plus_two();
    REQUIRE(m.is_connectivity_valid());

    SECTION("interior_edge")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 4));

        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(2)) == 0);
        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(7)) == 0);
        CHECK(fv_accessor.vector_attribute(0)[1] == 5);
        CHECK(fv_accessor.vector_attribute(1)[0] == 5);
        CHECK(fv_accessor.vector_attribute(3)[0] == 5);
        CHECK(fv_accessor.vector_attribute(5)[2] == 5);
        CHECK(fv_accessor.vector_attribute(6)[2] == 5);
        CHECK(fv_accessor.vector_attribute(4)[0] == 5);
    }
    SECTION("edge_to_boundary")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(4, 0, 0);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 4));

        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(0)) == 0);
        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(1)) == 0);

        CHECK(fv_accessor.vector_attribute(2)[0] == 0);
        CHECK(fv_accessor.vector_attribute(5)[2] == 0);
        CHECK(fv_accessor.vector_attribute(6)[2] == 0);
        CHECK(fv_accessor.vector_attribute(7)[0] == 0);
    }
    SECTION("edge_from_boundary_allowed")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 0);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        const bool success = !op(Simplex::edge(edge)).empty();
        CHECK(success);
    }
    SECTION("edge_from_boundary_prohibited")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 0);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        op.add_invariant(std::make_shared<InteriorVertexInvariant>(m));
        const bool fail = op(Simplex::edge(edge)).empty();
        CHECK(fail);
    }
    SECTION("boundary_edge")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 0));

        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(1)) == 0);

        CHECK(fv_accessor.vector_attribute(0)[2] == 1);
    }
    SECTION("boundary_edge_allowed")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        const bool success = !op(Simplex::edge(edge)).empty();
        CHECK(success);
    }
    SECTION("boundary_edge_prohibited")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        const bool fail = op(Simplex::edge(edge)).empty();
        CHECK(fail);
    }
}
