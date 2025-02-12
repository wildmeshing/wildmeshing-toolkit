#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TetCellSplit.hpp>
#include <wmtk/operations/composite/TetEdgeSwap.hpp>
#include <wmtk/operations/composite/TetFaceSwap.hpp>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/TetMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::simplex;
using namespace wmtk::operations;
using namespace wmtk::tests_3d;

using TM = TetMesh;
using MapResult = typename Eigen::Matrix<int64_t, Eigen::Dynamic, 1>::MapType;
#if defined(WMTK_ENABLE_HASH_UPDATE) 
using TMOE = decltype(std::declval<DEBUG_TetMesh>().get_tmoe(
    wmtk::Tuple(),
    std::declval<attribute::Accessor<int64_t>&>()));
#else
using TMOE = decltype(std::declval<DEBUG_TetMesh>().get_tmoe(
    wmtk::Tuple()));
#endif

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("tet_get_split_simplices_to_delete", "[operations][split][3d]")
{
    SECTION("single_tet")
    {
        const DEBUG_TetMesh m = single_tet();
        const Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);

        std::array<std::vector<int64_t>, 4> ids_to_delete =
            TMOE::get_split_simplices_to_delete(edge, m);

        // debug code
        // std::cout << "vertex: " << std::endl;
        // for (size_t i = 0; i < ids_to_delete[0].size(); i++) {
        //     std::cout << ids_to_delete[0][i] << std::endl;
        // }
        // std::cout << "edge: " << std::endl;
        // for (size_t i = 0; i < ids_to_delete[1].size(); i++) {
        //     std::cout << ids_to_delete[1][i] << std::endl;
        // }
        // std::cout << "face: " << std::endl;
        // for (size_t i = 0; i < ids_to_delete[2].size(); i++) {
        //     std::cout << ids_to_delete[2][i] << std::endl;
        // }
        // std::cout << "tet: " << std::endl;
        // for (size_t i = 0; i < ids_to_delete[3].size(); i++) {
        //     std::cout << ids_to_delete[3][i] << std::endl;
        // }

        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[2].size() == 2);
        REQUIRE(ids_to_delete[3].size() == 1);

        const int64_t edge_to_delete = ids_to_delete[1][0];
        CHECK(edge_to_delete == m.id(edge, PE));
        const int64_t tet_to_delete = ids_to_delete[3][0];
        CHECK(tet_to_delete == m.id(edge, PT));
        // TODO check faces
        const int64_t face_to_delete_1 = m.id(edge, PF);
        const int64_t face_to_delete_2 = m.id(m.switch_tuple(edge, PF), PF);

        // debugging code
        std::cout << "fid1: " << face_to_delete_1 << std::endl;
        std::cout << "fid2: " << face_to_delete_2 << std::endl;

        REQUIRE(
            std::find(ids_to_delete[2].begin(), ids_to_delete[2].end(), face_to_delete_1) !=
            ids_to_delete[2].end());
        REQUIRE(
            std::find(ids_to_delete[2].begin(), ids_to_delete[2].end(), face_to_delete_2) !=
            ids_to_delete[2].end());
    }
    SECTION("three_incident_tets")
    {
        const DEBUG_TetMesh m = three_incident_tets();
        const Tuple edge = m.edge_tuple_with_vs_and_t(2, 3, 1);

        std::array<std::vector<int64_t>, 4> ids_to_delete =
            TMOE::get_split_simplices_to_delete(edge, m);

        std::cout << "test three incident tets" << std::endl;

        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[2].size() == 4);
        REQUIRE(ids_to_delete[3].size() == 3);
    }
    SECTION("six_cycle_tets")
    {
        const DEBUG_TetMesh m = six_cycle_tets();
        const Tuple edge = m.edge_tuple_with_vs_and_t(2, 3, 5);

        std::array<std::vector<int64_t>, 4> ids_to_delete =
            TMOE::get_split_simplices_to_delete(edge, m);

        std::cout << "test six cycle tets" << std::endl;

        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[2].size() == 6);
        REQUIRE(ids_to_delete[3].size() == 6);
    }
}

TEST_CASE("tet_get_collapse_simplices_to_delete", "[operations][collapse][3D]")
{
    SECTION("single_tet")
    {
        const DEBUG_TetMesh m = single_tet();
        const Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);

        std::array<std::vector<int64_t>, 4> ids_to_delete =
            TMOE::get_collapse_simplices_to_delete(edge, m);

        // std::cout << "face: " << std::endl;
        // for (int i = 0; i < ids_to_delete[2].size(); i++) {
        //     std::cout << ids_to_delete[2][i] << std::endl;
        // }

        REQUIRE(ids_to_delete[0].size() == 1);
        REQUIRE(ids_to_delete[1].size() == 3);
        REQUIRE(ids_to_delete[2].size() == 3);
        REQUIRE(ids_to_delete[3].size() == 1);
    }
}

TEST_CASE("get_incident_tets_and_faces", "[operations][split][collapse][3d]")
{
    SECTION("single_tet")
    {
        DEBUG_TetMesh m = single_tet();
        const Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);


        auto executor = m.get_tmoe(edge);

        // std::array<std::vector<Tuple>, 2> incident_tets_and_faces =
        //     executor.get_incident_tets_and_faces(edge);

        // const auto& incident_tets = incident_tets_and_faces[0];
        // const auto& incident_faces = incident_tets_and_faces[1];

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);


        REQUIRE(incident_tets.size() == 1);
        REQUIRE(incident_faces.size() == 2);
        REQUIRE(m.id(incident_faces[0], PF) == m.id(edge, PF));
        REQUIRE(m.id(incident_faces[1], PF) == m.id(m.switch_face(edge), PF));
    }
    SECTION("one_ear")
    {
        DEBUG_TetMesh m = one_ear();
        const Tuple edge = m.face_tuple_with_vs_and_t(2, 3, 0, 0);

        auto executor = m.get_tmoe(edge);

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);

        REQUIRE(incident_tets.size() == 2);
        REQUIRE(incident_faces.size() == 3);
        REQUIRE(m.id(incident_faces[0], PF) == m.id(edge, PF));
        REQUIRE(m.id(incident_faces[2], PF) == m.id(m.switch_face(edge), PF));
    }
    SECTION("three_incident_tets_1")
    {
        DEBUG_TetMesh m = three_incident_tets();
        const Tuple edge = m.face_tuple_with_vs_and_t(2, 3, 0, 0);


        auto executor = m.get_tmoe(edge);

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);

        REQUIRE(incident_tets.size() == 3);
        REQUIRE(incident_faces.size() == 4);
        REQUIRE(m.id(incident_faces[0], PF) == m.id(edge, PF));
        REQUIRE(m.id(incident_faces[3], PF) == m.id(m.switch_face(edge), PF));
    }
    SECTION("three_incident_tets_2")
    {
        DEBUG_TetMesh m = three_incident_tets();
        const Tuple edge = m.face_tuple_with_vs_and_t(2, 3, 4, 1);


        auto executor = m.get_tmoe(edge);

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);

        REQUIRE(incident_tets.size() == 3);
        REQUIRE(incident_faces.size() == 4);
    }
    SECTION("three_incident_tets_3")
    {
        DEBUG_TetMesh m = three_incident_tets();
        const Tuple edge = m.edge_tuple_with_vs_and_t(2, 3, 2);


        auto executor = m.get_tmoe(edge);

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);

        REQUIRE(incident_tets.size() == 3);
        REQUIRE(incident_faces.size() == 4);
    }
    SECTION("six_cycle_tets")
    {
        DEBUG_TetMesh m = six_cycle_tets();
        const Tuple edge = m.edge_tuple_with_vs_and_t(2, 3, 0);

        auto executor = m.get_tmoe(edge);

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);


        REQUIRE(incident_tets.size() == 6);
        REQUIRE(incident_faces.size() == 6);
    }
}

TEST_CASE("tet_split_edge_single_tet", "[operations][split][3d]")
{
    DEBUG_TetMesh m = single_tet();

    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.face_tuple_with_vs_and_t(1, 2, 0, 0);
    EdgeSplit op(m);
    op(Simplex::edge(m, edge));
    REQUIRE(m.is_connectivity_valid());
}
TEST_CASE("tet_split_edge_one_ear", "[operations][split][3d]")
{
    DEBUG_TetMesh m = one_ear();

    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.face_tuple_with_vs_and_t(1, 2, 0, 0);
    EdgeSplit op(m);
    op(Simplex::edge(m, edge));
    REQUIRE(m.is_connectivity_valid());
}
TEST_CASE("tet_split_edge_two_ears", "[operations][split][3d]")
{
    DEBUG_TetMesh m = two_ears();

    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.face_tuple_with_vs_and_t(1, 2, 0, 0);
    EdgeSplit op(m);
    op(Simplex::edge(m, edge));
    REQUIRE(m.is_connectivity_valid());
}
TEST_CASE("tet_split_edge_three_incident_tets", "[operations][split][3d]")
{
    DEBUG_TetMesh m = three_incident_tets();

    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.face_tuple_with_vs_and_t(2, 3, 0, 1);
    EdgeSplit op(m);
    op(Simplex::edge(m, edge));
    REQUIRE(m.is_connectivity_valid());
}
TEST_CASE("tet_split_edge_six_cycle_tets", "[operations][split][3d]")
{
    DEBUG_TetMesh m = six_cycle_tets();

    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.face_tuple_with_vs_and_t(2, 3, 0, 0);
    EdgeSplit op(m);
    op(Simplex::edge(m, edge));
    REQUIRE(m.is_connectivity_valid());
}

TEST_CASE("tet_collapse_edge", "[operations][collapse][3d]")
{
    SECTION("one_ear")
    {
        DEBUG_TetMesh m = one_ear();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.face_tuple_with_vs_and_t(1, 2, 0, 0);
        EdgeCollapse op(m);
        op(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 1);
    }
    SECTION("two_ears")
    {
        DEBUG_TetMesh m = two_ears();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.face_tuple_with_vs_and_t(1, 2, 0, 0);
        EdgeCollapse op(m);
        op(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 2);
    }
    SECTION("three_incident_tets_1")
    {
        DEBUG_TetMesh m = three_incident_tets();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.face_tuple_with_vs_and_t(0, 4, 2, 1);
        EdgeCollapse op(m);
        op(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 2);
    }
    SECTION("three_incident_tets_2")
    {
        DEBUG_TetMesh m = three_incident_tets();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.face_tuple_with_vs_and_t(1, 2, 0, 0);
        EdgeCollapse op(m);
        op(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 2);
    }
    SECTION("six_cycle_tets_1")
    {
        DEBUG_TetMesh m = six_cycle_tets();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.face_tuple_with_vs_and_t(1, 2, 0, 0);
        EdgeCollapse op(m);
        op(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 4);
    }
    SECTION("six_cycle_tets_2")
    {
        DEBUG_TetMesh m = six_cycle_tets();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.face_tuple_with_vs_and_t(0, 4, 2, 1);
        EdgeCollapse op(m);
        op(Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 5);
    }
}

TEST_CASE("tet_edge_split", "[operations][split][3d]")
{
    using namespace operations;
    SECTION("single_tet")
    {
        //        0
        //       / \\ .
        //      /   \ \ .
        //     /     \  \ .
        //    /       \   \ 3
        //  1 --------- 2
        //
        DEBUG_TetMesh m = single_tet();
        EdgeSplit op(m);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_with_vs_and_t(1, 2, 0))),
                PrimitiveType::Vertex) == 3);
        auto res = op(Simplex::edge(m, m.edge_tuple_with_vs_and_t(1, 2, 0)));
        CHECK(!res.empty());
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 5);
        CHECK(m.get_all(PrimitiveType::Edge).size() == 9);
        CHECK(m.get_all(PrimitiveType::Triangle).size() == 7);
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 2);

        const auto res_tuple = res.front().tuple();
        CHECK(m.id(res_tuple, PrimitiveType::Vertex) == 4);
        CHECK(m.id(m.switch_vertex(res_tuple), PrimitiveType::Vertex) == 2);
        CHECK(m.id(m.switch_vertex(m.switch_edge(res_tuple)), PrimitiveType::Vertex) == 3);
        CHECK(
            m.id(m.switch_vertex(m.switch_edge(m.switch_face(res_tuple))), PrimitiveType::Vertex) ==
            0);

        //  CHECK(
        //      m.id(
        //          m.switch_vertex(m.switch_edge(m.switch_face(m.switch_tetrahedron(
        //              m.switch_face(m.switch_edge(m.switch_vertex(res_tuple))))))),
        //          PrimitiveType::Vertex) == 2);

        const auto [spine_edge0, spine_edge1] = EdgeSplit::new_spine_edges(m, res_tuple);
        CHECK(m.id(spine_edge0, PrimitiveType::Vertex) == 4);
        CHECK(m.id(spine_edge1, PrimitiveType::Vertex) == 4);
        CHECK(m.id(m.switch_vertex(spine_edge0), PrimitiveType::Vertex) == 2);

        CHECK(m.id(m.switch_vertex(spine_edge1), PrimitiveType::Vertex) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(spine_edge0)), PrimitiveType::Vertex) == 3);
        CHECK(m.id(m.switch_vertex(m.switch_edge(spine_edge1)), PrimitiveType::Vertex) == 3);
    }
    SECTION("two_ears")
    {
        //  5 --------- 0 ---------- 4
        //   \  \      / \\        /
        //    \      \/   \ \     /
        //     \     /    \\  \  /
        //      \   /       \  \\ 3
        //        1 --------- 2/      tuple edge 1-2
        //
        DEBUG_TetMesh m = two_ears();
        EdgeSplit op(m);
        auto res = op(Simplex::edge(m, m.edge_tuple_with_vs_and_t(1, 2, 0)));
        CHECK(!res.empty());
        auto res_tuple = res.front().tuple();
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 7);
        CHECK(m.get_all(PrimitiveType::Edge).size() == 15);
        CHECK(m.get_all(PrimitiveType::Triangle).size() == 13);
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 4);
        CHECK(m.id(res_tuple, PrimitiveType::Vertex) == 6);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(
                    m.switch_face(m.switch_tetrahedron(m.switch_face(m.switch_edge(res_tuple)))))),
                PrimitiveType::Vertex) == 1);
        const auto [spine_edge0, spine_edge1] = EdgeSplit::new_spine_edges(m, res_tuple);
        CHECK(m.id(spine_edge0, PrimitiveType::Vertex) == 6);
        CHECK(m.id(spine_edge1, PrimitiveType::Vertex) == 6);
        CHECK(m.id(m.switch_vertex(spine_edge0), PrimitiveType::Vertex) == 2);

        CHECK(m.id(m.switch_vertex(spine_edge1), PrimitiveType::Vertex) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(spine_edge0)), PrimitiveType::Vertex) == 3);
        CHECK(m.id(m.switch_vertex(m.switch_edge(spine_edge1)), PrimitiveType::Vertex) == 3);
    }
}

TEST_CASE("tet_edge_collapse", "[operations][collapse][3d]")
{
    using namespace operations;
    SECTION("two_ears_collapse_mid")
    {
        //  5 --------- 0 ---------- 4
        //   \  \      / \\        /
        //    \      \/   \ \     /
        //     \     /    \\  \  /
        //      \   /       \  \\ 3
        //        1 --------- 2/      tuple edge 1-2
        //
        DEBUG_TetMesh m = two_ears();
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_with_vs_and_t(1, 2, 0))),
                PrimitiveType::Vertex) == 3);
        auto res = op(Simplex::edge(m, m.edge_tuple_with_vs_and_t(1, 2, 0)));
        CHECK(!res.empty());
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 5);
        CHECK(m.get_all(PrimitiveType::Edge).size() == 9);
        CHECK(m.get_all(PrimitiveType::Triangle).size() == 7);
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 2);
        auto res_tuple = res.front().tuple();
        CHECK(m.id(res_tuple, PrimitiveType::Vertex) == 2);
        CHECK(m.id(m.switch_vertex(m.switch_edge(res_tuple)), PrimitiveType::Vertex) == 0);
        CHECK(
            m.id(m.switch_vertex(m.switch_edge(m.switch_face(res_tuple))), PrimitiveType::Vertex) ==
            4);
    }
    SECTION("two_ears_collapse_right")
    {
        //  5 --------- 0 ---------- 4
        //   \  \      / \\        /
        //    \      \/   \ \     /
        //     \     /    \\  \  /
        //      \   /       \  \\ 3
        //        1 --------- 2/      tuple edge 1-2
        //
        DEBUG_TetMesh m = two_ears();
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_with_vs_and_t(2, 4, 1))),
                PrimitiveType::Vertex) == 0);
        auto res = op(Simplex::edge(m, m.edge_tuple_with_vs_and_t(2, 4, 1)));
        CHECK(!res.empty());
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 5);
        CHECK(m.get_all(PrimitiveType::Edge).size() == 9);
        CHECK(m.get_all(PrimitiveType::Triangle).size() == 7);
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 2);
        auto res_tuple = res.front().tuple();
        CHECK(m.id(res_tuple, PrimitiveType::Vertex) == 4);
        CHECK(m.id(m.switch_vertex(m.switch_edge(res_tuple)), PrimitiveType::Vertex) == 3);
        CHECK(
            m.id(m.switch_vertex(m.switch_edge(m.switch_face(res_tuple))), PrimitiveType::Vertex) ==
            1);
    }
}


TEST_CASE("tet_tet_split", "[operations][split][collapse][3d]")
{
    using namespace operations::composite;
    SECTION("single_tet")
    {
        //        0
        //       / \\ .
        //      /   \ \ .
        //     /     \  \ .
        //    /       \   \ 3
        //  1 --------- 2
        //
        DEBUG_TetMesh m = single_tet();
        TetCellSplit op(m);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_with_vs_and_t(1, 2, 0))),
                PrimitiveType::Vertex) == 3);
        auto s = Simplex::tetrahedron(m, m.edge_tuple_with_vs_and_t(1, 2, 0));

        auto res = op(s);
        CHECK(!res.empty());
        auto res_tuple = res.front().tuple();
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 4);
        CHECK(m.id(res_tuple, PrimitiveType::Vertex) == 1);
        CHECK(m.id(m.switch_vertex(res_tuple), PrimitiveType::Vertex) == 6);
        CHECK(m.id(m.switch_vertex(m.switch_edge(res_tuple)), PrimitiveType::Vertex) == 2);
    }
    SECTION("two_ears")
    {
        //  5 --------- 0 ---------- 4
        //   \  \      / \\        /
        //    \      \/   \ \     /
        //     \     /    \\  \  /
        //      \   /       \  \\ 3
        //        1 --------- 2/      tuple edge 1-2
        //
        // split 0-2-3-4 with edge 2-3
        DEBUG_TetMesh m = two_ears();
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_with_vs_and_t(2, 3, 0))),
                PrimitiveType::Vertex) == 1);
        TetCellSplit op(m);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_with_vs_and_t(2, 3, 0))),
                PrimitiveType::Vertex) == 1);
        auto res = op(Simplex::tetrahedron(m, m.edge_tuple_with_vs_and_t(2, 3, 0)));
        CHECK(!res.empty());
        auto res_tuple = res.front().tuple();
        CHECK(m.id(res_tuple, PrimitiveType::Vertex) == 2);
        CHECK(m.id(m.switch_vertex(res_tuple), PrimitiveType::Vertex) == 8);
        CHECK(m.id(m.switch_vertex(m.switch_edge(res_tuple)), PrimitiveType::Vertex) == 3);

        /*
        Simplex v(PrimitiveType::Vertex, m.switch_vertex(res_tuple));
        auto sc = simplex::open_star(m, v);
        {
            std::vector<Tuple> modified_tuples = op.modified_primitives(PrimitiveType::Tetrahedron);
            for (const Simplex& s : sc.get_simplices(PrimitiveType::Tetrahedron)) {
                bool t_exist = false;
                int times = 0;
                for (const Tuple& t : modified_tuples) {
                    if (m.id(t, PrimitiveType::Tetrahedron) ==
                        m.id(s.tuple(), PrimitiveType::Tetrahedron)) {
                        t_exist = true;
                        break;
                    }
                }
                CHECK(t_exist);
            }
        }
        {
            std::vector<Tuple> modified_tuples = op.modified_primitives(PrimitiveType::Face);
            for (const Simplex& s : sc.get_simplices(PrimitiveType::Face)) {
                bool t_exist = false;
                int times = 0;
                for (const Tuple& t : modified_tuples) {
                    if (m.id(t, PrimitiveType::Face) == m.id(s.tuple(), PrimitiveType::Face)) {
                        t_exist = true;
                        break;
                    }
                }
                CHECK(t_exist);
            }
        }
        {
            std::vector<Tuple> modified_tuples = op.modified_primitives(PrimitiveType::Edge);
            for (const Simplex& s : sc.get_simplices(PrimitiveType::Edge)) {
                bool t_exist = false;
                int times = 0;
                for (const Tuple& t : modified_tuples) {
                    if (m.id(t, PrimitiveType::Edge) == m.id(s.tuple(), PrimitiveType::Edge)) {
                        t_exist = true;
                        break;
                    }
                }
                CHECK(t_exist);
            }
        }
        {
            std::vector<Tuple> modified_tuples = op.modified_primitives(PrimitiveType::Vertex);
            for (const Simplex& s : sc.get_simplices(PrimitiveType::Vertex)) {
                bool t_exist = false;
                int times = 0;
                for (const Tuple& t : modified_tuples) {
                    if (m.id(t, PrimitiveType::Vertex) == m.id(s.tuple(), PrimitiveType::Vertex)) {
                        t_exist = true;
                        break;
                    }
                }
                CHECK(t_exist);
            }
        }
        */
    }
}

TEST_CASE("tet_split_with_tags", "[operations][split][3d]")
{
    using namespace operations;

    const int64_t embedding_tag_value = 0;
    const int64_t input_tag_value = 1;
    const int64_t split_tag_value = 2;

    SECTION("single_tet")
    {
        //        0
        //       / \\ .
        //      /   \ \ .
        //     /     \  \ .
        //    /       \   \ 3
        //  1 --------- 2
        //
        DEBUG_TetMesh m = single_tet();
        Eigen::MatrixXd V(4, 3);
        V.row(0) << 0.5, 0.86, 0;
        V.row(1) << 0, 0, 0;
        V.row(2) << 1.0, 0, -1.0;
        V.row(3) << 1.0, 0, 1.0;
        wmtk::mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
        wmtk::attribute::MeshAttributeHandle vertex_tag_handle = m.register_attribute<int64_t>(
            "vertex_tag",
            wmtk::PrimitiveType::Vertex,
            1,
            false,
            embedding_tag_value);
        wmtk::attribute::MeshAttributeHandle edge_tag_handle = m.register_attribute<int64_t>(
            "edge_tag",
            wmtk::PrimitiveType::Edge,
            1,
            false,
            embedding_tag_value);
        wmtk::attribute::MeshAttributeHandle todo_tag_handle =
            m.register_attribute<int64_t>("todo_tag", wmtk::PrimitiveType::Tetrahedron, 1);
        wmtk::attribute::Accessor<int64_t> acc_edge_tag =
            m.create_accessor<int64_t>(edge_tag_handle);
        acc_edge_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(1, 2, 0)) = 5;
        wmtk::attribute::Accessor<int64_t> acc_todo_tag =
            m.create_accessor<int64_t>(todo_tag_handle);
        acc_todo_tag.scalar_attribute(m.edge_tuple_with_vs_and_t(1, 2, 0)) = 1;

        composite::TetCellSplit op(m);
        op.add_invariant(std::make_shared<TodoInvariant>(m, todo_tag_handle.as<int64_t>()));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        op.collapse().set_new_attribute_strategy(todo_tag_handle);
        op.collapse().set_new_attribute_strategy(edge_tag_handle);
        op.collapse().set_new_attribute_strategy(vertex_tag_handle);
        op.collapse().set_new_attribute_strategy(pos_handle);
        op.split().set_new_attribute_strategy(todo_tag_handle);
        op.split().set_new_attribute_strategy(edge_tag_handle);
        op.split().set_new_attribute_strategy(vertex_tag_handle);
        op.split().set_new_attribute_strategy(pos_handle);

        auto res = op(Simplex::tetrahedron(m, m.edge_tuple_with_vs_and_t(1, 2, 0)));

        CHECK(!res.empty());
        auto return_tuple = res.front().tuple();
        CHECK(m.id(return_tuple, PrimitiveType::Vertex) == 1);
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 5);
        CHECK(m.get_all(PrimitiveType::Edge).size() == 10);
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 4);
    }

    SECTION("one_ear")
    {
        //        0 ---------- 4
        //       / \\        // \ .
        //      /   \ \     //   \ .
        //     /     \  \  //     \ .
        //    /       \   \3       \ .
        //  1 --------- 2/ -------- 5   tuple edge 2-3
        //    \       /  /\ \      / .
        //     \     / /   \\     / .
        //      \   //      \\   / .
        //       \ //        \  / .
        //        6 -----------7
        //
        DEBUG_TetMesh m = six_cycle_tets();
        Eigen::MatrixXd V(8, 3);
        V.row(0) << 0.5, 0.86, 0;
        V.row(1) << 0, 0, 0;
        V.row(2) << 1.0, 0, 1.0;
        V.row(3) << 1.0, 0, -1.0;
        V.row(4) << 1.5, 0.86, 0;
        V.row(5) << 2, 0, 0;
        V.row(6) << 0.5, -0.86, 0;
        V.row(7) << 1.5, -0.86, 0;
        wmtk::mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
        wmtk::attribute::MeshAttributeHandle vertex_tag_handle = m.register_attribute<int64_t>(
            "vertex_tag",
            wmtk::PrimitiveType::Vertex,
            1,
            false,
            embedding_tag_value);
        wmtk::attribute::MeshAttributeHandle edge_tag_handle = m.register_attribute<int64_t>(
            "edge_tag",
            wmtk::PrimitiveType::Edge,
            1,
            false,
            embedding_tag_value);
        wmtk::attribute::MeshAttributeHandle todo_tag_handle =
            m.register_attribute<int64_t>("todo_tag", wmtk::PrimitiveType::Tetrahedron, 1);
        wmtk::attribute::Accessor<int64_t> acc_todo_tag =
            m.create_accessor<int64_t>(todo_tag_handle);
        acc_todo_tag.scalar_attribute(m.get_all(PrimitiveType::Tetrahedron)[0]) = 1;
        acc_todo_tag.scalar_attribute(m.get_all(PrimitiveType::Tetrahedron)[3]) = 1;

        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_with_vs_and_t(1, 2, 0))),
                PrimitiveType::Vertex) == 3);

        composite::TetCellSplit op(m);
        op.add_invariant(std::make_shared<TodoInvariant>(m, todo_tag_handle.as<int64_t>()));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        // used the default strategy, so the tag is not updated.
        auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        op.collapse().set_new_attribute_strategy(todo_tag_handle);
        op.collapse().set_new_attribute_strategy(edge_tag_handle);
        op.collapse().set_new_attribute_strategy(vertex_tag_handle);
        op.collapse().set_new_attribute_strategy(pos_handle);
        op.split().set_new_attribute_strategy(todo_tag_handle);
        op.split().set_new_attribute_strategy(edge_tag_handle);
        op.split().set_new_attribute_strategy(vertex_tag_handle);
        op.split().set_new_attribute_strategy(pos_handle);

        CHECK(op(Simplex::tetrahedron(m, m.edge_tuple_with_vs_and_t(7, 2, 5))).empty());

        auto res = op(Simplex::tetrahedron(m, m.edge_tuple_with_vs_and_t(1, 2, 0)));

        CHECK(!res.empty());
        auto return_tuple = res.front().tuple();

        CHECK(m.id(return_tuple, PrimitiveType::Vertex) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(return_tuple)), PrimitiveType::Vertex) == 2);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(
                    m.switch_face(m.switch_tetrahedron(m.switch_edge(return_tuple))))),
                PrimitiveType::Vertex) == 3);
        CHECK(
            acc_todo_tag.scalar_attribute(m.switch_tetrahedron(
                m.switch_face(m.switch_tetrahedron(m.switch_edge(return_tuple))))) == 1);
    }
}

TEST_CASE("tetmesh_edge_swap", "[operations][swap][split][collapse][3d]")
{
    using namespace operations::composite;
    using namespace tests_3d;

    SECTION("swap32-0")
    {
        DEBUG_TetMesh m = three_cycle_tets();
        TetEdgeSwap op(m, 0);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.face_tuple_with_vs_and_t(0, 1, 2, 0);
        auto ret_faces = op(Simplex::edge(m, edge));
        CHECK(ret_faces.size() == 1);
        REQUIRE(m.is_connectivity_valid());
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 2);
    }
    SECTION("swap32-1")
    {
        DEBUG_TetMesh m = three_cycle_tets();
        TetEdgeSwap op(m, 1);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.face_tuple_with_vs_and_t(0, 1, 2, 0);
        auto ret_faces = op(Simplex::edge(m, edge));
        CHECK(ret_faces.size() == 1);
        REQUIRE(m.is_connectivity_valid());
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 2);
    }
    SECTION("swap32-2")
    {
        DEBUG_TetMesh m = three_cycle_tets();
        TetEdgeSwap op(m, 2);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.face_tuple_with_vs_and_t(0, 1, 2, 0);
        auto ret_faces = op(Simplex::edge(m, edge));
        CHECK(ret_faces.size() == 1);
        REQUIRE(m.is_connectivity_valid());
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 2);
    }
    SECTION("swap44-0")
    {
        DEBUG_TetMesh m = four_cycle_tets();
        TetEdgeSwap op(m, 0);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.face_tuple_with_vs_and_t(0, 1, 2, 3);
        auto ret_edges = op(Simplex::edge(m, edge));
        CHECK(ret_edges.size() == 1);
        REQUIRE(m.is_connectivity_valid());
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 4);
    }
    SECTION("swap44-1")
    {
        DEBUG_TetMesh m = four_cycle_tets();
        TetEdgeSwap op(m, 1);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.face_tuple_with_vs_and_t(0, 1, 2, 3);
        auto ret_edges = op(Simplex::edge(m, edge));
        CHECK(ret_edges.size() == 1);
        REQUIRE(m.is_connectivity_valid());
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 4);
    }
}

TEST_CASE("tetmesh_face_swap", "[operations][swap][split][collapse][3d]")
{
    using namespace operations::composite;
    using namespace tests_3d;

    SECTION("one_ear")
    {
        DEBUG_TetMesh m = one_ear();
        TetFaceSwap op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        REQUIRE(m.is_connectivity_valid());

        const Tuple face = m.face_tuple_with_vs_and_t(0, 2, 3, 0);
        auto ret_edges = op(Simplex::face(m, face));
        CHECK(ret_edges.size() == 1);
        REQUIRE(m.is_connectivity_valid());
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 3);
        CHECK(m.id(ret_edges.front().tuple(), PV) == 4);
        CHECK(m.id(m.switch_vertex(ret_edges.front().tuple()), PV) == 1);
    }
    SECTION("six_cycle_tets")
    {
        DEBUG_TetMesh m = six_cycle_tets();
        TetFaceSwap op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        REQUIRE(m.is_connectivity_valid());

        const Tuple face = m.face_tuple_with_vs_and_t(2, 3, 0, 0);
        auto ret_edges = op(Simplex::face(m, face));
        CHECK(ret_edges.size() == 1);
        REQUIRE(m.is_connectivity_valid());
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 7);
        CHECK(m.id(ret_edges.front().tuple(), PV) == 4);
        CHECK(m.id(m.switch_vertex(ret_edges.front().tuple()), PV) == 1);
    }
}
