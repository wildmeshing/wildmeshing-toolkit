
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
#include "../tools/is_free.hpp"
#include "../tools/redirect_logger_to_cout.hpp"

using namespace wmtk;
using namespace wmtk::simplex;
using namespace wmtk::tests;
using namespace operations;

using TM = TriMesh;
using MapResult = typename Eigen::Matrix<int64_t, Eigen::Dynamic, 1>::MapType;
#if defined(WMTK_ENABLE_HASH_UPDATE)
using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe(
    wmtk::Tuple(),
    std::declval<wmtk::attribute::Accessor<int64_t>&>()));
#else
using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe(wmtk::Tuple()));
#endif

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
        const Tuple edge = m.edge_tuple_with_vs_and_t(4, 5, 2);
        EdgeCollapse collapse(m);
        collapse(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());

        auto& fv_accessor = m.create_base_accessor<int64_t>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 4));

        REQUIRE(face_flag_accessor.index_access().is_active(2) == false);
        REQUIRE(face_flag_accessor.index_access().is_active(7) == false);
        CHECK(fv_accessor.vector_attribute(0)[1] == 5);
        CHECK(fv_accessor.vector_attribute(1)[0] == 5);
        CHECK(fv_accessor.vector_attribute(3)[0] == 5);
        CHECK(fv_accessor.vector_attribute(5)[2] == 5);
        CHECK(fv_accessor.vector_attribute(6)[2] == 5);
        CHECK(fv_accessor.vector_attribute(4)[0] == 5);
    }
    SECTION("edge_to_boundary")
    {
        const Tuple edge = m.edge_tuple_with_vs_and_t(4, 0, 0);
        EdgeCollapse collapse(m);
        collapse(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());

        auto& fv_accessor = m.create_base_accessor<int64_t>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 4));

        REQUIRE(face_flag_accessor.index_access().is_active(0) == false);
        REQUIRE(face_flag_accessor.index_access().is_active(1) == false);

        CHECK(fv_accessor.vector_attribute(2)[0] == 0);
        CHECK(fv_accessor.vector_attribute(5)[2] == 0);
        CHECK(fv_accessor.vector_attribute(6)[2] == 0);
        CHECK(fv_accessor.vector_attribute(7)[0] == 0);
    }
    SECTION("edge_from_boundary_allowed")
    {
        const Tuple edge = m.edge_tuple_with_vs_and_t(0, 4, 0);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        const bool success = !op(Simplex::edge(m, edge)).empty();
        CHECK(success);
    }
    SECTION("edge_from_boundary_prohibited")
    {
        const Tuple edge = m.edge_tuple_with_vs_and_t(0, 4, 0);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        op.add_invariant(std::make_shared<InteriorVertexInvariant>(m));
        const bool fail = op(Simplex::edge(m, edge)).empty();
        CHECK(fail);
    }
    SECTION("boundary_edge")
    {
        const Tuple edge = m.edge_tuple_with_vs_and_t(0, 1, 1);
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());

        auto& fv_accessor = m.create_base_accessor<int64_t>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 0));

        REQUIRE(face_flag_accessor.index_access().is_active(1) == false);

        CHECK(fv_accessor.vector_attribute(0)[2] == 1);
    }
    SECTION("boundary_edge_allowed")
    {
        const Tuple edge = m.edge_tuple_with_vs_and_t(0, 1, 1);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        const bool success = !op(Simplex::edge(m, edge)).empty();
        CHECK(success);
    }
    SECTION("boundary_edge_prohibited")
    {
        const Tuple edge = m.edge_tuple_with_vs_and_t(0, 1, 1);

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
    SECTION("interior")
    {
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_with_vs_and_t(4, 5, 2);
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        auto res = op(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();

#if defined(WMTK_ENABLE_HASH_UPDATE)
        REQUIRE(m.is_valid_with_hash(ret));
#else
        REQUIRE(m.is_valid(ret));
#endif
        REQUIRE(m.is_connectivity_valid());
        // CHECK(op.is_return_tuple_from_left_ear() == false);

        CHECK(m.id(ret, PV) == 5);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(ret, PF) == 1);
    }
    SECTION("from_boundary")
    {
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_with_vs_and_t(3, 4, 0);
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

        const Tuple edge = m.edge_tuple_with_vs_and_t(4, 3, 0);
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

        const Tuple t = m.edge_tuple_with_vs_and_t(0, 1, 0);
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
                if (!m.is_valid(t)) {
                    continue;
                }
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
TEST_CASE("get_collapse_simplices_to_delete", "[operations][collapse][2D]")
{
    SECTION("interior_edge")
    {
        const DEBUG_TriMesh m = edge_region();
        Tuple edge = m.edge_tuple_with_vs_and_t(4, 5, 2);

        std::array<std::vector<int64_t>, 3> ids_to_delete =
            TMOE::get_collapse_simplices_to_delete(edge, m);

        REQUIRE(ids_to_delete[0].size() == 1);
        REQUIRE(ids_to_delete[1].size() == 3);
        REQUIRE(ids_to_delete[2].size() == 2);

        // V
        const int64_t vertex_to_delete = ids_to_delete[0][0];
        CHECK(vertex_to_delete == m.id(edge, PV));

        // E
        std::set<int64_t> eid_expected;
        eid_expected.insert(m.id(edge, PE));
        eid_expected.insert(m.id(m.switch_edge(edge), PE));
        eid_expected.insert(m.id(m.switch_edge(m.switch_face(edge)), PE));

        std::set<int64_t> eid_actual;
        for (const int64_t& e : ids_to_delete[1]) {
            CHECK(eid_expected.find(e) != eid_expected.end());
            eid_actual.insert(e);
        }
        CHECK(eid_actual.size() == eid_expected.size());

        // F
        std::set<int64_t> fid_expected;
        fid_expected.insert(m.id(edge, PF));
        fid_expected.insert(m.id(m.switch_face(edge), PF));

        std::set<int64_t> fid_actual;
        for (const int64_t& f : ids_to_delete[2]) {
            CHECK(fid_expected.find(f) != fid_expected.end());
            fid_actual.insert(f);
        }
        CHECK(fid_actual.size() == fid_expected.size());
    }
    SECTION("boundary_edge")
    {
        const DEBUG_TriMesh m = edge_region();
        Tuple edge = m.edge_tuple_with_vs_and_t(7, 8, 6);

        std::array<std::vector<int64_t>, 3> ids_to_delete =
            TMOE::get_collapse_simplices_to_delete(edge, m);

        REQUIRE(ids_to_delete[0].size() == 1);
        REQUIRE(ids_to_delete[1].size() == 2);
        REQUIRE(ids_to_delete[2].size() == 1);

        // V
        const int64_t vertex_to_delete = ids_to_delete[0][0];
        CHECK(vertex_to_delete == m.id(edge, PV));

        // E
        std::set<int64_t> eid_expected;
        eid_expected.insert(m.id(edge, PE));
        eid_expected.insert(m.id(m.switch_edge(edge), PE));

        std::set<int64_t> eid_actual;
        for (const int64_t& e : ids_to_delete[1]) {
            CHECK(eid_expected.find(e) != eid_expected.end());
            eid_actual.insert(e);
        }
        CHECK(eid_actual.size() == eid_expected.size());

        // F
        const int64_t face_to_delete = ids_to_delete[2][0];
        CHECK(face_to_delete == m.id(edge, PF));
    }
    SECTION("interior_edge_incident_to_boundary")
    {
        const DEBUG_TriMesh m = edge_region();
        Tuple edge = m.edge_tuple_with_vs_and_t(7, 4, 5);

        std::array<std::vector<int64_t>, 3> sc_to_delete =
            TMOE::get_collapse_simplices_to_delete(edge, m);

        REQUIRE(sc_to_delete[0].size() == 1);
        REQUIRE(sc_to_delete[1].size() == 3);
        REQUIRE(sc_to_delete[2].size() == 2);

        // V
        const int64_t vertex_to_delete = sc_to_delete[0][0];
        CHECK(vertex_to_delete == m.id(edge, PV));

        // E
        std::set<int64_t> eid_expected;
        eid_expected.insert(m.id(edge, PE));
        eid_expected.insert(m.id(m.switch_edge(edge), PE));
        eid_expected.insert(m.id(m.switch_edge(m.switch_face(edge)), PE));

        std::set<int64_t> eid_actual;
        for (const int64_t& e : sc_to_delete[1]) {
            CHECK(eid_expected.find(e) != eid_expected.end());
            eid_actual.insert(e);
        }
        CHECK(eid_actual.size() == eid_expected.size());

        // F
        std::set<int64_t> fid_expected;
        fid_expected.insert(m.id(edge, PF));
        fid_expected.insert(m.id(m.switch_face(edge), PF));

        std::set<int64_t> fid_actual;
        for (const int64_t& f : sc_to_delete[2]) {
            CHECK(fid_expected.find(f) != fid_expected.end());
            fid_actual.insert(f);
        }
        CHECK(fid_actual.size() == fid_expected.size());
    }
    SECTION("free")
    {
        const DEBUG_TriMesh m = []() {
            TriMesh m;
            m.initialize_free(20);
            return m;
        }();
        REQUIRE(m.is_free());
        for (Tuple edge : m.get_all(PrimitiveType::Edge)) {
            std::array<std::vector<int64_t>, 3> ids_to_delete =
                TMOE::get_collapse_simplices_to_delete(edge, m);


            REQUIRE(ids_to_delete[0].size() == 3);
            REQUIRE(ids_to_delete[1].size() == 3);
            REQUIRE(ids_to_delete[2].size() == 1);

            // compare expected face ids with the actual ones that should be deleted
            std::set<int64_t> fid_expected;
            fid_expected.insert(m.id(edge, PF));

            std::set<int64_t> fid_actual;
            for (const int64_t& f : ids_to_delete[2]) {
                CHECK(fid_expected.find(f) != fid_expected.end());
                fid_actual.insert(f);
            }
            CHECK(fid_actual.size() == fid_expected.size());
        }
    }
}

TEST_CASE("collapse_no_topology_trimesh", "[operations][collapse]")
{
    const int64_t initial_size = 20;
    DEBUG_TriMesh m = [](int64_t size) {
        TriMesh m;
        m.initialize_free(size);
        return m;
    }(initial_size);
    int64_t size = initial_size;
    size_t count = 0;
    for (Tuple edge : m.get_all(PrimitiveType::Triangle)) {
        // spdlog::info("Count: {}", count++);
        EdgeCollapse op(m);
        REQUIRE(!op(simplex::Simplex(m, PrimitiveType::Edge, edge)).empty());
        size--;
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(is_free(m));
        CHECK(m.get_all(PrimitiveType::Triangle).size() == size);
    }
}
