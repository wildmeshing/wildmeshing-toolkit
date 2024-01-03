#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <numeric>
#include <set>
#include <wmtk/Accessor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
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
    std::declval<Accessor<int64_t>&>()));

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;


TEST_CASE("swap_edge", "[operations][swap][2D]")
{
    using namespace operations::composite;

    SECTION("counter_clockwise")
    {
        DEBUG_TriMesh m = interior_edge();
        TriEdgeSwap op(m);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));


        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto res = op(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
        REQUIRE(m.is_connectivity_valid());

        CHECK(m.id(ret, PV) == 4);
        CHECK(m.id(m.switch_vertex(ret), PV) == 0);

        auto fv_accessor = m.create_const_base_accessor<int64_t>(m.f_handle(PrimitiveType::Vertex));
        auto f5_fv = fv_accessor.vector_attribute(5);
        CHECK(f5_fv[0] == 1);
        CHECK(f5_fv[1] == 4);
        CHECK(f5_fv[2] == 0);
        auto f6_fv = fv_accessor.vector_attribute(6);
        CHECK(f6_fv[0] == 0);
        CHECK(f6_fv[1] == 4);
        CHECK(f6_fv[2] == 2);
    }
    SECTION("clockwise")
    {
        DEBUG_TriMesh m = interior_edge();
        TriEdgeSwap op(m);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 2);
        auto res = op(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
        REQUIRE(m.is_connectivity_valid());

        CHECK(m.id(ret, PV) == 0);
        CHECK(m.id(m.switch_vertex(ret), PV) == 4);

        auto fv_accessor = m.create_const_base_accessor<int64_t>(m.f_handle(PrimitiveType::Vertex));
        auto f5_fv = fv_accessor.vector_attribute(5);
        auto f6_fv = fv_accessor.vector_attribute(6);
        CHECK(f5_fv[0] == 0);
        CHECK(f5_fv[1] == 1);
        CHECK(f5_fv[2] == 4);
        CHECK(f6_fv[0] == 0);
        CHECK(f6_fv[1] == 4);
        CHECK(f6_fv[2] == 2);
    }
    SECTION("single_triangle_fail")
    {
        DEBUG_TriMesh m = single_triangle();
        TriEdgeSwap op(m);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        REQUIRE(m.is_connectivity_valid());
        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        REQUIRE(op(Simplex::edge(edge)).empty());
        REQUIRE(m.is_connectivity_valid());
    }
    SECTION("tetrahedron_fail")
    {
        DEBUG_TriMesh m = tetrahedron();
        TriEdgeSwap op(m);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(2, 1, 1);
        REQUIRE(op(Simplex::edge(edge)).empty());
        REQUIRE(m.is_connectivity_valid());
    }
}

TEST_CASE("split_face", "[operations][split][2D]")
{
    using namespace operations::composite;
    SECTION("split_single_triangle")
    {
        //         0
        //        / \ .
        //       2   1
        //      /  0  \ .
        //     /       \ .
        //  1  ----0---- 2
        //
        // this case covered the on boundary case
        DEBUG_TriMesh m = single_triangle();
        const Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        auto result = op(Simplex::face(f));
        bool is_success = !result.empty();
        CHECK(is_success);
        const Tuple ret = result.front().tuple();
        CHECK(m.get_all(PV).size() == 4);
        CHECK(!m.is_boundary_vertex(ret));
        CHECK(!m.is_boundary_edge(ret));
        CHECK(!m.is_boundary_edge(m.switch_edge(ret)));
        CHECK(m.id(ret, PV) == 4);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret)), PV) == 2);
        CHECK(simplex::link(m, simplex::Simplex::vertex(ret)).simplex_vector().size() == 6);
    }

    SECTION("split_face_in_quad")
    {
        //  3--1--- 0
        //   |     / \ .
        //   2 f1 /2   1
        //   |  0/ f0  \ .
        //   |  /       \ .
        //  1  ----0---- 2
        //
        DEBUG_TriMesh m = quad();
        Tuple f = m.edge_tuple_between_v1_v2(1, 0, 1);
        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        const auto res = op(Simplex::face(f));
        const bool is_success = !res.empty();
        CHECK(is_success);
        CHECK(m.get_all(PV).size() == 5);
        const auto ret_tuple = res.front().tuple();
        CHECK(m.id(ret_tuple, PV) == 5);
        CHECK(m.id(m.switch_vertex(ret_tuple), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret_tuple)), PV) == 0);
    }

    SECTION("split_with_attribute")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ / \ /
        //    7---8---9
        DEBUG_TriMesh m = edge_region_with_position();
        MeshAttributeHandle<double> pos_handle = m.get_attribute_handle<double>("vertices", PV);

        MeshAttributeHandle<int64_t> attri_handle =
            m.register_attribute<int64_t>("test_attribute", PF, 1);

        MeshAttributeHandle<double> v2_handle = m.register_attribute<double>("vertices2", PV, 1);

        Accessor<int64_t> acc_attri = m.create_accessor<int64_t>(attri_handle);
        for (const Tuple& f : m.get_all(PF)) {
            acc_attri.scalar_attribute(f) = 1;
        }

        composite::TriFaceSplit op(m);

        {
            auto new_split =
                std::make_shared<wmtk::operations::SplitNewAttributeStrategy<double>>(pos_handle);
            new_split->set_strategy(operations::SplitBasicStrategy::Default);
            new_split->set_rib_strategy(operations::SplitRibBasicStrategy::Default);
            op.split().set_new_attribute_strategy(pos_handle, new_split);
        }

        {
            // or if you want to swap out the existing behavior with a new behavior
            auto new_split =
                std::make_shared<wmtk::operations::SplitNewAttributeStrategy<double>>(pos_handle);
            new_split->set_strategy(operations::SplitBasicStrategy::Default);
            new_split->set_rib_strategy(operations::SplitRibBasicStrategy::Default);
            op.split().set_new_attribute_strategy(v2_handle, new_split);
        }

        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op.split().set_new_attribute_strategy(
            attri_handle,
            wmtk::operations::SplitBasicStrategy::Copy);
        op.collapse().set_new_attribute_strategy(attri_handle);
        op.collapse().set_new_attribute_strategy(pos_handle);
        op.collapse().set_new_attribute_strategy(v2_handle);


        const Tuple f0 = m.face_tuple_from_vids(3, 4, 0);
        CHECK(!op(Simplex::face(f0)).empty());
        const Tuple f1 = m.face_tuple_from_vids(8, 9, 5);
        CHECK(!op(Simplex::face(f1)).empty());
        const Tuple f2 = m.face_tuple_from_vids(4, 8, 5);
        CHECK(!op(Simplex::face(f2)).empty());

        for (const Tuple& t : m.get_all(PF)) {
            CHECK(acc_attri.scalar_attribute(t) == 1);
        }

        if (false) {
            io::Cache cache("");
            wmtk::io::ParaviewWriter writer(
                cache.get_cache_path() / "split_in_diamond_with_attribute_result",
                "vertices",
                m,
                false,
                false,
                true,
                false);
            m.serialize(writer);
        }
    }
    SECTION("split_single_triangle_at_mid_point")
    {
        //         0
        //        / \ .
        //       2   1
        //      /  0  \ .
        //     /       \ .
        //  1  ----0---- 2
        //
        // this case covered the on boundary case
        // V.row(0) << 0, 0, 0;
        // V.row(1) << 1, 0, 0;
        // V.row(2) << 0.5, 0.866, 0;
        DEBUG_TriMesh m = single_equilateral_triangle(3);
        Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        MeshAttributeHandle<double> pos_handle = m.get_attribute_handle<double>("vertices", PV);

        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op.split().set_new_attribute_strategy(pos_handle);
        op.collapse().set_new_attribute_strategy(
            pos_handle,
            operations::CollapseBasicStrategy::CopyOther);

        auto res = op(Simplex::face(f));
        bool is_success = !res.empty();
        CHECK(is_success);
        Tuple ret = res.front().tuple();
        CHECK(m.get_all(PV).size() == 4);
        CHECK(!m.is_boundary_vertex(ret));
        CHECK(!m.is_boundary_edge(ret));
        CHECK(!m.is_boundary_edge(m.switch_edge(ret)));
        CHECK(m.id(ret, PV) == 4);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret)), PV) == 2);
        CHECK(
            simplex::link(m, Simplex::vertex(ret)).simplex_vector(PrimitiveType::Vertex).size() ==
            3);
        Accessor<double> acc_pos = m.create_accessor<double>(pos_handle);
        CHECK(acc_pos.vector_attribute(ret).x() == 0.375);
        CHECK(acc_pos.vector_attribute(m.switch_vertex(ret)).x() == 1);
    }
    SECTION("split_single_triangle_at_mid_point_with_tag_embedding_on")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ / \ /
        //    7---8---9
        DEBUG_TriMesh m = edge_region();
        Tuple f = m.edge_tuple_between_v1_v2(3, 4, 0);

        MeshAttributeHandle<int64_t> todo_handle =
            m.register_attribute<int64_t>("todo_face", PF, 1);

        Accessor<int64_t> acc_todo = m.create_accessor<int64_t>(todo_handle);
        acc_todo.scalar_attribute(f) = 1;

        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op.add_invariant(std::make_shared<TodoInvariant>(m, todo_handle));
        op.split().set_new_attribute_strategy(
            todo_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::None);
        op.collapse().set_new_attribute_strategy(todo_handle, CollapseBasicStrategy::None);

        CHECK(!op(Simplex::face(f)).empty());

        CHECK(m.get_all(PF).size() == 12);
        for (const Tuple& t : m.get_all(PF)) {
            CHECK(acc_todo.scalar_attribute(t) == 0);
        }
    }

    SECTION("split_single_triangle_at_mid_point_with_tag_embedding_off")
    {
        DEBUG_TriMesh m = single_triangle();

        const Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        MeshAttributeHandle<int64_t> todo_handle =
            m.register_attribute<int64_t>("todo_face", PF, 1);
        MeshAttributeHandle<int64_t> edge_tag_handle =
            m.register_attribute<int64_t>("edge_tag", PE, 1);
        MeshAttributeHandle<int64_t> vertex_tag_handle =
            m.register_attribute<int64_t>("vertex_tag", PV, 1);
        Accessor<int64_t> acc_todo = m.create_accessor<int64_t>(todo_handle);
        Accessor<int64_t> acc_edge_tag = m.create_accessor<int64_t>(edge_tag_handle);
        Accessor<int64_t> acc_vertex_tag = m.create_accessor<int64_t>(vertex_tag_handle);
        acc_todo.scalar_attribute(f) = 1;

        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(0, 1, 0)) = 1;
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 2;
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(2, 0, 0)) = 3;

        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(0, 1, 0)) = 1;
        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 2;
        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(2, 0, 0)) = 3;

        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        op.split().set_new_attribute_strategy(
            todo_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::None);
        op.collapse().set_new_attribute_strategy(todo_handle, CollapseBasicStrategy::None);

        op.split().set_new_attribute_strategy(
            edge_tag_handle,
            SplitBasicStrategy::Copy,
            SplitRibBasicStrategy::None);
        op.collapse().set_new_attribute_strategy(edge_tag_handle, CollapseBasicStrategy::None);

        op.split().set_new_attribute_strategy(
            vertex_tag_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::None);
        op.collapse().set_new_attribute_strategy(
            vertex_tag_handle,
            wmtk::operations::CollapseBasicStrategy::None);


        op.add_invariant(std::make_shared<TodoInvariant>(m, todo_handle));
        const auto res = op(Simplex::face(f));

        CHECK(!res.empty());
        const Tuple& return_tuple = res.front().tuple();
        const Tuple v0 = m.switch_vertex(m.switch_edge(m.switch_face(return_tuple)));
        const Tuple v1 = m.switch_vertex(return_tuple);
        const Tuple v2 = m.switch_vertex(m.switch_edge(return_tuple));

        const int64_t id0 = m.id_vertex(return_tuple);
        const int64_t id1 = m.id_vertex(m.switch_vertex(return_tuple));
        const int64_t id2 = m.id_vertex(m.switch_vertex(m.switch_edge(return_tuple)));
        CHECK(acc_vertex_tag.scalar_attribute(return_tuple) == 0);
        CHECK(acc_vertex_tag.scalar_attribute(v0) == 1);
        CHECK(acc_vertex_tag.scalar_attribute(v1) == 2);
        CHECK(acc_vertex_tag.scalar_attribute(v2) == 3);

        CHECK(acc_edge_tag.scalar_attribute(return_tuple) == 0);

        const Tuple e01 = m.switch_edge(m.switch_face(m.switch_vertex(return_tuple)));
        const Tuple e12 = m.switch_edge(m.switch_vertex(return_tuple));
        const Tuple e20 =
            m.switch_edge(m.switch_face(m.switch_vertex(m.switch_edge(return_tuple))));
        CHECK(acc_edge_tag.scalar_attribute(e01) == 1);
        CHECK(acc_edge_tag.scalar_attribute(e12) == 2);
        CHECK(acc_edge_tag.scalar_attribute(e20) == 3);

        CHECK(m.get_all(PF).size() == 3);
        for (const Tuple& t : m.get_all(PF)) {
            CHECK(acc_todo.scalar_attribute(t) == 0);
        }
    }

    SECTION("should fail with todo tag 0")
    {
        DEBUG_TriMesh m = single_equilateral_triangle(3);

        const Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        MeshAttributeHandle<int64_t> todo_handle =
            m.register_attribute<int64_t>("todo_face", PF, 1);


        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op.add_invariant(std::make_shared<TodoInvariant>(m, todo_handle));

        op.split().set_new_attribute_strategy(
            todo_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::None);

        CHECK(op(Simplex::face(f)).empty());
    }
}
