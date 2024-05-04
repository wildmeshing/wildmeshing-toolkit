
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <numeric>
#include <set>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/operations/composite/TriFaceSplit.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
#include "../tools/redirect_logger_to_cout.hpp"

using namespace wmtk;
using namespace wmtk::simplex;
using namespace wmtk::tests;
using namespace operations;

using TM = TriMesh;
using MapResult = typename Eigen::Matrix<int64_t, Eigen::Dynamic, 1>::MapType;
using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe(
    wmtk::Tuple(),
    std::declval<attribute::Accessor<int64_t>&>()));

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;
//////////// COLLAPSE TESTS ////////////

TEST_CASE("collapse_edge", "[operations][collapse][2D]")
{
    using namespace operations;

    DEBUG_TriMesh m = hex_plus_two();
    REQUIRE(m.is_connectivity_valid());

    auto face_flag_accessor = m.get_flag_accessor(PrimitiveType::Triangle);


    SECTION("interior_edge")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
        wmtk::attribute::Accessor<int64_t> hash_accessor = m.get_cell_hash_accessor();
        EdgeCollapse collapse(m);
        collapse(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<int64_t>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 4));

        REQUIRE(face_flag_accessor.scalar_attribute(m.tuple_from_face_id(2)) == 0);
        REQUIRE(face_flag_accessor.scalar_attribute(m.tuple_from_face_id(7)) == 0);
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
        wmtk::attribute::Accessor<int64_t> hash_accessor = m.get_cell_hash_accessor();
        EdgeCollapse collapse(m);
        collapse(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<int64_t>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 4));

        REQUIRE(face_flag_accessor.scalar_attribute(m.tuple_from_face_id(0)) == 0);
        REQUIRE(face_flag_accessor.scalar_attribute(m.tuple_from_face_id(1)) == 0);

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

        const bool success = !op(Simplex::edge(m, edge)).empty();
        CHECK(success);
    }
    SECTION("edge_from_boundary_prohibited")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 0);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        op.add_invariant(std::make_shared<InteriorVertexInvariant>(m));
        const bool fail = op(Simplex::edge(m, edge)).empty();
        CHECK(fail);
    }
    SECTION("boundary_edge")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);
        wmtk::attribute::Accessor<int64_t> hash_accessor = m.get_cell_hash_accessor();
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<int64_t>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 0));

        REQUIRE(face_flag_accessor.scalar_attribute(m.tuple_from_face_id(1)) == 0);

        CHECK(fv_accessor.vector_attribute(0)[2] == 1);
    }
    SECTION("boundary_edge_allowed")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        const bool success = !op(Simplex::edge(m, edge)).empty();
        CHECK(success);
    }
    SECTION("boundary_edge_prohibited")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        const bool fail = op(Simplex::edge(m, edge)).empty();
        CHECK(fail);
    }
}

TEST_CASE("collapse_return_tuple", "[operations][collapse][2D]")
{
    DEBUG_TriMesh m = edge_region();
    wmtk::attribute::Accessor<int64_t> hash_accessor = m.get_cell_hash_accessor();
    SECTION("interior")
    {
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        auto res = op(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();

        REQUIRE(m.is_valid_slow(ret));
        REQUIRE(m.is_connectivity_valid());
        // CHECK(op.is_return_tuple_from_left_ear() == false);

        CHECK(m.id(ret, PV) == 5);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(ret, PF) == 1);
    }
    SECTION("from_boundary")
    {
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(3, 4, 0);
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        auto res = op(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
        REQUIRE(m.is_connectivity_valid());
        // CHECK(op.is_return_tuple_from_left_ear() == false);

        CHECK(m.id(ret, PV) == 4);
        CHECK(m.id(m.switch_vertex(ret), PV) == 0);
        CHECK(m.id(ret, PF) == 1);
    }
    SECTION("to_boundary")
    {
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(4, 3, 0);
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        auto res = op(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
        REQUIRE(m.is_connectivity_valid());

        CHECK(m.id(ret, PV) == 3);
        CHECK(m.id(m.switch_vertex(ret), PV) == 0);
        CHECK(m.id(ret, PF) == 1);
    }
}
TEST_CASE("split_edge_operation_with_tag", "[operations][split][2D]")
{
    //  3--1--- 0
    //   |     / \ .
    //   2 f1 /2   1
    //   |  0/ f0  \ .
    //   |  /       \ .
    //  1  ----0---- 2
    //     \        /
    //      \  f2  /
    //       \    /
    //        \  /
    //         4
    using namespace operations;

    Eigen::MatrixXd V(5, 3);
    V << 0, 0, 0, -1, -1, 0, 1, -1, 0, -1, 1, 0, 1, -1, 0;
    DEBUG_TriMesh m = interior_edge();
    wmtk::mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);

    wmtk::attribute::MeshAttributeHandle edge_tag_handle =
        m.register_attribute<int64_t>("edge_tag", PE, 1);
    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    SECTION("single_split")
    {
        EdgeSplit op(m);

        op.set_new_attribute_strategy(
            edge_tag_handle,
            SplitBasicStrategy::Copy,
            SplitRibBasicStrategy::None);
        op.set_new_attribute_strategy(pos_handle);

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);
        {
            auto acc_tag_e = m.create_accessor<int64_t>(edge_tag_handle);
            acc_tag_e.scalar_attribute(t) = 1;
        }

        const auto res = op(Simplex::edge(m, t));
        CHECK(!res.empty());

        const Tuple spine1 = res.front().tuple();
        const Tuple rib1 = m.switch_edge(m.switch_face(spine1));
        const Tuple spine2 = m.switch_edge(m.switch_face(rib1));
        const Tuple rib2 = m.switch_edge(m.switch_face(spine2));
        {
            auto acc_tag_e = m.create_accessor<int64_t>(edge_tag_handle);
            CHECK(acc_tag_e.scalar_attribute(spine1) == 1);
            CHECK(acc_tag_e.scalar_attribute(spine2) == 1);
            CHECK(acc_tag_e.scalar_attribute(rib1) == 0);
            CHECK(acc_tag_e.scalar_attribute(rib2) == 0);
        }
    }

    wmtk::attribute::MeshAttributeHandle vertex_tag_handle =
        m.register_attribute<int64_t>(std::string("vertex_tag"), PV, 1);

    wmtk::attribute::MeshAttributeHandle todo_handle =
        m.register_attribute<int64_t>(std::string("todo_tag"), PE, 1);

    EdgeSplit op(m);
    op.set_new_attribute_strategy(pos_handle);
    op.set_new_attribute_strategy(vertex_tag_handle);
    op.set_new_attribute_strategy(
        edge_tag_handle,
        SplitBasicStrategy::Copy,
        SplitRibBasicStrategy::None);

    op.set_new_attribute_strategy(
        todo_handle,
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::None);


    op.add_invariant(std::make_shared<TodoInvariant>(m, todo_handle.as<int64_t>()));

    SECTION("no_todo_edges")
    {
        for (const Tuple& t : m.get_all(PV)) {
            CHECK(op(Simplex::edge(m, t)).empty());
        }
    }

    SECTION("interior_todo_with_tags")
    {
        wmtk::attribute::Accessor<int64_t> acc_todo = m.create_accessor<int64_t>(todo_handle);
        wmtk::attribute::Accessor<int64_t> acc_tag_e = m.create_accessor<int64_t>(edge_tag_handle);
        wmtk::attribute::Accessor<int64_t> acc_tag_v =
            m.create_accessor<int64_t>(vertex_tag_handle);
        for (const Tuple& e : m.get_all(PE)) {
            if (!m.is_boundary_edge(e)) {
                acc_tag_e.scalar_attribute(e) = 1;
                acc_todo.scalar_attribute(e) = 1;
            }
        }
        int success_num = 0;
        // should perform two iterations to split the two interior edges once
        for (int i = 0; i < 5; i++) {
            for (const Tuple& t : m.get_all(PE)) {
                const auto res = op(Simplex::edge(m, t));
                if (!res.empty()) {
                    const Tuple spine1 = res.front().tuple();
                    const Tuple rib1 = m.switch_edge(m.switch_face(spine1));
                    const Tuple spine2 = m.switch_edge(m.switch_face(rib1));
                    const Tuple rib2 = m.switch_edge(m.switch_face(spine2));
                    // todo marks should be removed
                    CHECK(acc_todo.scalar_attribute(spine1) == 0);
                    CHECK(acc_todo.scalar_attribute(spine2) == 0);
                    CHECK(acc_todo.scalar_attribute(rib1) == 0);
                    CHECK(acc_todo.scalar_attribute(rib2) == 0);

                    // splitted edges should be tagged
                    CHECK(acc_tag_e.scalar_attribute(spine1) == 1);
                    CHECK(acc_tag_e.scalar_attribute(spine2) == 1);
                    // new edges should not be tagged
                    CHECK(acc_tag_e.scalar_attribute(rib1) == 0);
                    CHECK(acc_tag_e.scalar_attribute(rib2) == 0);

                    // TODO test for vertex attributes

                    success_num++;
                }
            }
        }
        CHECK(success_num == 2);
    }
}
