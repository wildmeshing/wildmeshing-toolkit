#include <catch2/catch_test_macros.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/operations/AttributeTransferStrategy.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::invariants;
TEST_CASE("split_edge_attr_transfer", "[operations][split][2D]")
{
    using namespace operations;
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /
    //    7---8
    DEBUG_TriMesh m = hex_plus_two_with_position();


    // this handel already has default behaviors so lets leave it alone
    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    //
    //


    auto edge_length_handle = m.register_attribute<double>("edge_length", PrimitiveType::Edge, 1);

    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };

    std::shared_ptr el_strategy =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_handle,
            pos_handle,
            compute_edge_length);

    {
        // initialize
        auto edges = m.get_all(PrimitiveType::Edge);
        for (const auto& e : edges) {
            el_strategy->run(simplex::Simplex::edge(e));
        }
    }

    auto pos_acc = pos_handle.create_const_accessor();
    auto el_acc = edge_length_handle.create_const_accessor();

    auto check_lengths = [&]() {
        auto edges = m.get_all(PrimitiveType::Edge);
        for (const auto& e : edges) {
            // el_strategy->update(simplex::Simplex::edge(e));


            auto v0 = e;
            auto v1 = m.switch_vertex(e);

            auto pos0 = pos_acc.const_vector_attribute(v0);
            auto pos1 = pos_acc.const_vector_attribute(v1);

            double len = (pos0 - pos1).norm();

            double len2 = el_acc.const_scalar_attribute(e);

            CHECK(len == len2);
        }
    };


    check_lengths();
    auto hash_accessor = m.get_cell_hash_accessor();

    // add strategy (api can be cleaned up of course)
    m.m_transfer_strategies.emplace_back(el_strategy);

    EdgeSplit op(m);
    op.add_transfer_strategy(el_strategy);
    op.set_standard_strategy(edge_length_handle);
    op.set_standard_strategy(pos_handle);
    Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
    bool success = !op(Simplex::edge(edge)).empty();
    CHECK(success);
    check_lengths();

    Tuple edge2 = m.edge_tuple_between_v1_v2(3, 0, 0);
    success = !op(Simplex::edge(edge2)).empty();
    CHECK(success);
    check_lengths();

    Tuple edge3 = m.edge_tuple_between_v1_v2(4, 7, 6);
    success = !op(Simplex::edge(edge3)).empty();
    CHECK(success);
    REQUIRE(m.is_connectivity_valid());
    check_lengths();

    Tuple edge4 = m.edge_tuple_between_v1_v2(4, 9, 8);
    success = !op(Simplex::edge(edge4)).empty();
    CHECK(success);
    check_lengths();

    Tuple edge5 = m.edge_tuple_between_v1_v2(5, 6, 4);
    success = !op(Simplex::edge(edge5)).empty();
    CHECK(success);
    check_lengths();
}
TEST_CASE("collapse_edge_attr_transfer", "[operations][collapse][2D]")
{
    using namespace operations;

    DEBUG_TriMesh m = hex_plus_two_with_position();
    REQUIRE(m.is_connectivity_valid());

    // this handel already has default behaviors so lets leave it alone
    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    //
    //


    auto edge_length_handle = m.register_attribute<double>("edge_length", PrimitiveType::Edge, 1);

    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };

    std::shared_ptr el_strategy =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_handle,
            pos_handle,
            compute_edge_length);


    auto pos_acc = pos_handle.create_const_accessor();
    auto el_acc = edge_length_handle.create_const_accessor();

    {
        // initialize
        auto edges = m.get_all(PrimitiveType::Edge);
        for (const auto& e : edges) {
            el_strategy->run(simplex::Simplex::edge(e));
        }
    }

    auto check_lengths = [&]() {
        auto edges = m.get_all(PrimitiveType::Edge);
        for (const auto& e : edges) {
            auto v0 = e;
            auto v1 = m.switch_vertex(e);

            auto pos0 = pos_acc.const_vector_attribute(v0);
            auto pos1 = pos_acc.const_vector_attribute(v1);

            double len = (pos0 - pos1).norm();

            double len2 = el_acc.const_scalar_attribute(e);

            CHECK(len == len2);
        }
    };
    EdgeCollapse op(m);
    op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
    op.add_transfer_strategy(el_strategy);
    op.set_standard_strategy(edge_length_handle);
    op.set_standard_strategy(pos_handle);

    SECTION("interior_edge")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
        const bool success = !op(Simplex::edge(edge)).empty();
        CHECK(success);
        check_lengths();
    }
    SECTION("edge_to_boundary")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(4, 0, 0);
        const bool success = !op(Simplex::edge(edge)).empty();
        CHECK(success);
        check_lengths();
    }
    SECTION("edge_from_boundary_allowed")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 0);


        const bool success = !op(Simplex::edge(edge)).empty();
        CHECK(success);
        check_lengths();
    }
    SECTION("boundary_edge")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);
        const bool success = !op(Simplex::edge(edge)).empty();
        CHECK(success);
        check_lengths();
    }
}
