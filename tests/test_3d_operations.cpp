#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/Accessor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/operations/tet_mesh/EdgeCollapse.hpp>
#include <wmtk/operations/tet_mesh/EdgeSplit.hpp>
#include <wmtk/operations/tet_mesh/EdgeSplitWithTags.hpp>
#include <wmtk/operations/tet_mesh/TetSplit.hpp>
#include <wmtk/operations/tet_mesh/TetSplitWithTags.hpp>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/TetMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests_3d;

using TM = TetMesh;
using MapResult = typename Eigen::Matrix<long, Eigen::Dynamic, 1>::MapType;
using TMOE = decltype(std::declval<DEBUG_TetMesh>().get_tmoe(
    wmtk::Tuple(),
    std::declval<Accessor<long>&>()));

constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("get_split_simplices_to_delete", "[operations][split][3d]")
{
    SECTION("single_tet")
    {
        const DEBUG_TetMesh m = single_tet();
        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);

        std::array<std::vector<long>, 4> ids_to_delete =
            TMOE::get_split_simplices_to_delete(edge, m);

        // debug code
        std::cout << "vertex: " << std::endl;
        for (size_t i = 0; i < ids_to_delete[0].size(); i++) {
            std::cout << ids_to_delete[0][i] << std::endl;
        }
        std::cout << "edge: " << std::endl;
        for (size_t i = 0; i < ids_to_delete[1].size(); i++) {
            std::cout << ids_to_delete[1][i] << std::endl;
        }
        std::cout << "face: " << std::endl;
        for (size_t i = 0; i < ids_to_delete[2].size(); i++) {
            std::cout << ids_to_delete[2][i] << std::endl;
        }
        std::cout << "tet: " << std::endl;
        for (size_t i = 0; i < ids_to_delete[3].size(); i++) {
            std::cout << ids_to_delete[3][i] << std::endl;
        }

        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[2].size() == 2);
        REQUIRE(ids_to_delete[3].size() == 1);

        const long edge_to_delete = ids_to_delete[1][0];
        CHECK(edge_to_delete == m._debug_id(edge, PE));
        const long tet_to_delete = ids_to_delete[3][0];
        CHECK(tet_to_delete == m._debug_id(edge, PT));
        // TODO check faces
        const long face_to_delete_1 = m._debug_id(edge, PF);
        const long face_to_delete_2 = m._debug_id(m.switch_tuple(edge, PF), PF);

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
        const Tuple edge = m.edge_tuple_between_v1_v2(2, 3, 1);

        std::array<std::vector<long>, 4> ids_to_delete =
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
        const Tuple edge = m.edge_tuple_between_v1_v2(2, 3, 5);

        std::array<std::vector<long>, 4> ids_to_delete =
            TMOE::get_split_simplices_to_delete(edge, m);

        std::cout << "test six cycle tets" << std::endl;

        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[2].size() == 6);
        REQUIRE(ids_to_delete[3].size() == 6);
    }
}

TEST_CASE("get_collapse_simplices_to_delete", "[operations][collapse][3D]")
{
    SECTION("single_tet")
    {
        const DEBUG_TetMesh m = single_tet();
        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);

        std::array<std::vector<long>, 4> ids_to_delete =
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
        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);


        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

        // std::array<std::vector<Tuple>, 2> incident_tets_and_faces =
        //     executor.get_incident_tets_and_faces(edge);

        // const auto& incident_tets = incident_tets_and_faces[0];
        // const auto& incident_faces = incident_tets_and_faces[1];

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);


        REQUIRE(incident_tets.size() == 1);
        REQUIRE(incident_faces.size() == 2);
        REQUIRE(m._debug_id(incident_faces[0], PF) == m._debug_id(edge, PF));
        REQUIRE(m._debug_id(incident_faces[1], PF) == m._debug_id(m.switch_face(edge), PF));
    }
    SECTION("one_ear")
    {
        DEBUG_TetMesh m = one_ear();
        const Tuple edge = m.edge_tuple_between_v1_v2(2, 3, 0, 0);

        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);

        REQUIRE(incident_tets.size() == 2);
        REQUIRE(incident_faces.size() == 3);
        REQUIRE(m._debug_id(incident_faces[0], PF) == m._debug_id(edge, PF));
        REQUIRE(m._debug_id(incident_faces[2], PF) == m._debug_id(m.switch_face(edge), PF));
    }
    SECTION("three_incident_tets_1")
    {
        DEBUG_TetMesh m = three_incident_tets();
        const Tuple edge = m.edge_tuple_between_v1_v2(2, 3, 0, 0);


        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);

        REQUIRE(incident_tets.size() == 3);
        REQUIRE(incident_faces.size() == 4);
        REQUIRE(m._debug_id(incident_faces[0], PF) == m._debug_id(edge, PF));
        REQUIRE(m._debug_id(incident_faces[3], PF) == m._debug_id(m.switch_face(edge), PF));
    }
    SECTION("three_incident_tets_2")
    {
        DEBUG_TetMesh m = three_incident_tets();
        const Tuple edge = m.edge_tuple_between_v1_v2(2, 3, 4, 1);


        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);

        REQUIRE(incident_tets.size() == 3);
        REQUIRE(incident_faces.size() == 4);
    }
    SECTION("three_incident_tets_3")
    {
        DEBUG_TetMesh m = three_incident_tets();
        const Tuple edge = m.edge_tuple_between_v1_v2(2, 3, 2);


        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);

        REQUIRE(incident_tets.size() == 3);
        REQUIRE(incident_faces.size() == 4);
    }
    SECTION("six_cycle_tets")
    {
        DEBUG_TetMesh m = six_cycle_tets();
        const Tuple edge = m.edge_tuple_between_v1_v2(2, 3, 0);

        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

        const auto [incident_tets, incident_faces] = executor.get_incident_tets_and_faces(edge);


        REQUIRE(incident_tets.size() == 6);
        REQUIRE(incident_faces.size() == 6);
    }
}

TEST_CASE("tet_split_edge_single_tet", "[operations][split][3d]")
{
    DEBUG_TetMesh m = single_tet();
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();

    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0, 0);
    m.split_edge(edge, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
}
TEST_CASE("tet_split_edge_one_ear", "[operations][split][3d]")
{
    DEBUG_TetMesh m = one_ear();
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();

    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0, 0);
    m.split_edge(edge, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
}
TEST_CASE("tet_split_edge_two_ears", "[operations][split][3d]")
{
    DEBUG_TetMesh m = two_ears();
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();

    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0, 0);
    m.split_edge(edge, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
}
TEST_CASE("tet_split_edge_three_incident_tets", "[operations][split][3d]")
{
    DEBUG_TetMesh m = three_incident_tets();
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();

    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_between_v1_v2(2, 3, 0, 1);
    m.split_edge(edge, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
}
TEST_CASE("tet_split_edge_six_cycle_tets", "[operations][split][3d]")
{
    DEBUG_TetMesh m = six_cycle_tets();
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();

    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_between_v1_v2(2, 3, 0, 0);
    m.split_edge(edge, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
}

TEST_CASE("collapse_edge", "[operation][collapse][3d]")
{
    SECTION("one_ear")
    {
        DEBUG_TetMesh m = one_ear();
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0, 0);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 1);
    }
    SECTION("two_ears")
    {
        DEBUG_TetMesh m = two_ears();
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0, 0);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 2);
    }
    SECTION("three_incident_tets_1")
    {
        DEBUG_TetMesh m = three_incident_tets();
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 2, 1);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 2);
    }
    SECTION("three_incident_tets_2")
    {
        DEBUG_TetMesh m = three_incident_tets();
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0, 0);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 2);
    }
    SECTION("six_cycle_tets_1")
    {
        DEBUG_TetMesh m = six_cycle_tets();
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0, 0);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 4);
    }
    SECTION("six_cycle_tets_2")
    {
        DEBUG_TetMesh m = six_cycle_tets();
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 2, 1);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 5);
    }
}

TEST_CASE("tet_edge_split", "[operation][split][3d]")
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
        OperationSettings<tet_mesh::EdgeSplit> settings;
        settings.initialize_invariants(m);
        CHECK(settings.are_invariants_initialized());
        tet_mesh::EdgeSplit op(m, m.edge_tuple_between_v1_v2(1, 2, 0), settings);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_between_v1_v2(1, 2, 0))),
                PrimitiveType::Vertex) == 3);
        CHECK(op.name().compare("tet_mesh_split_edge") == 0);
        CHECK(op());
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 5);
        CHECK(m.get_all(PrimitiveType::Edge).size() == 9);
        CHECK(m.get_all(PrimitiveType::Face).size() == 7);
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 2);
        CHECK(op.return_tuple() == op.new_vertex());
        CHECK(m.id(op.return_tuple(), PrimitiveType::Vertex) == 1);
        CHECK(m.id(m.switch_vertex(op.return_tuple()), PrimitiveType::Vertex) == 4);
        CHECK(m.id(m.switch_vertex(m.switch_edge(op.return_tuple())), PrimitiveType::Vertex) == 3);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.switch_face(op.return_tuple()))),
                PrimitiveType::Vertex) == 0);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.switch_face(m.switch_tetrahedron(
                    m.switch_face(m.switch_edge(m.switch_vertex(op.return_tuple()))))))),
                PrimitiveType::Vertex) == 2);
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
        OperationSettings<tet_mesh::EdgeSplit> settings;
        settings.initialize_invariants(m);
        CHECK(settings.are_invariants_initialized());
        tet_mesh::EdgeSplit op(m, m.edge_tuple_between_v1_v2(1, 2, 0), settings);
        CHECK(op.name().compare("tet_mesh_split_edge") == 0);
        CHECK(op());
        CHECK(op.return_tuple() == op.new_vertex());
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 7);
        CHECK(m.get_all(PrimitiveType::Edge).size() == 15);
        CHECK(m.get_all(PrimitiveType::Face).size() == 13);
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 4);
        CHECK(m.id(op.return_tuple(), PrimitiveType::Vertex) == 1);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.switch_face(
                    m.switch_tetrahedron(m.switch_face(m.switch_edge(op.return_tuple())))))),
                PrimitiveType::Vertex) == 5);
    }
}

TEST_CASE("tet_edge_collapse", "[operation][collapse][3d]")
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
        OperationSettings<tet_mesh::EdgeCollapse> settings;
        settings.initialize_invariants(m);
        CHECK(settings.are_invariants_initialized());
        tet_mesh::EdgeCollapse op(m, m.edge_tuple_between_v1_v2(1, 2, 0), settings);
        CHECK(op.name().compare("tet_mesh_collapse_edge") == 0);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_between_v1_v2(1, 2, 0))),
                PrimitiveType::Vertex) == 3);
        CHECK(op());
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 5);
        CHECK(m.get_all(PrimitiveType::Edge).size() == 9);
        CHECK(m.get_all(PrimitiveType::Face).size() == 7);
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 2);
        CHECK(m.id(op.return_tuple(), PrimitiveType::Vertex) == 2);
        CHECK(m.id(m.switch_vertex(m.switch_edge(op.return_tuple())), PrimitiveType::Vertex) == 0);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.switch_face(op.return_tuple()))),
                PrimitiveType::Vertex) == 4);
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
        OperationSettings<tet_mesh::EdgeCollapse> settings;
        settings.initialize_invariants(m);
        CHECK(settings.are_invariants_initialized());
        tet_mesh::EdgeCollapse op(m, m.edge_tuple_between_v1_v2(2, 4, 1), settings);
        CHECK(op.name().compare("tet_mesh_collapse_edge") == 0);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_between_v1_v2(2, 4, 1))),
                PrimitiveType::Vertex) == 0);
        CHECK(op());
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 5);
        CHECK(m.get_all(PrimitiveType::Edge).size() == 9);
        CHECK(m.get_all(PrimitiveType::Face).size() == 7);
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 2);
        CHECK(m.id(op.return_tuple(), PrimitiveType::Vertex) == 4);
        CHECK(m.id(m.switch_vertex(m.switch_edge(op.return_tuple())), PrimitiveType::Vertex) == 3);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.switch_face(op.return_tuple()))),
                PrimitiveType::Vertex) == 1);
    }
}

TEST_CASE("tet_tet_split", "[operation][split][collapse][3d]")
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
        OperationSettings<tet_mesh::TetSplit> settings;
        settings.initialize_invariants(m);
        CHECK(settings.are_invariants_initialized());
        tet_mesh::TetSplit op(m, m.edge_tuple_between_v1_v2(1, 2, 0), settings);
        CHECK(op.name().compare("tet_mesh_split_tet") == 0);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_between_v1_v2(1, 2, 0))),
                PrimitiveType::Vertex) == 3);
        CHECK(op());
        CHECK(op.return_tuple() == op.new_vertex());
        CHECK(m.get_all(PrimitiveType::Tetrahedron).size() == 4);
        CHECK(m.id(op.return_tuple(), PrimitiveType::Vertex) == 1);
        CHECK(m.id(m.switch_vertex(op.return_tuple()), PrimitiveType::Vertex) == 6);
        CHECK(m.id(m.switch_vertex(m.switch_edge(op.return_tuple())), PrimitiveType::Vertex) == 2);
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
                m.switch_vertex(m.switch_edge(m.edge_tuple_between_v1_v2(2, 3, 0))),
                PrimitiveType::Vertex) == 1);
        OperationSettings<tet_mesh::TetSplit> settings;
        settings.initialize_invariants(m);
        CHECK(settings.are_invariants_initialized());
        tet_mesh::TetSplit op(m, m.edge_tuple_between_v1_v2(2, 3, 0), settings);
        CHECK(op.name().compare("tet_mesh_split_tet") == 0);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_between_v1_v2(2, 3, 0))),
                PrimitiveType::Vertex) == 1);
        CHECK(op.name().compare("tet_mesh_split_tet") == 0);
        CHECK(op());
        CHECK(op.return_tuple().same_ids(op.new_vertex()));
        CHECK(op.return_tuple() == op.new_vertex());
        CHECK(m.id(op.return_tuple(), PrimitiveType::Vertex) == 2);
        CHECK(m.id(m.switch_vertex(op.return_tuple()), PrimitiveType::Vertex) == 8);
        CHECK(m.id(m.switch_vertex(m.switch_edge(op.return_tuple())), PrimitiveType::Vertex) == 3);

        Simplex v(PrimitiveType::Vertex, m.switch_vertex(op.return_tuple()));
        auto sc = SimplicialComplex::open_star(m, v);
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
    }
}

TEST_CASE("tet_edge_split_with_tags", "[operation][split][3d]")
{
    using namespace operations;

    const long embedding_tag_value = 0;
    const long input_tag_value = 1;
    const long split_tag_value = 2;
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
        MeshAttributeHandle<double> pos_handle =
            m.get_attribute_handle<double>("vertices", wmtk::PrimitiveType::Vertex);
        MeshAttributeHandle<long> vertex_tag_handle = m.register_attribute<long>(
            "vertex_tag",
            wmtk::PrimitiveType::Vertex,
            1,
            false,
            embedding_tag_value);
        MeshAttributeHandle<long> edge_tag_handle = m.register_attribute<long>(
            "edge_tag",
            wmtk::PrimitiveType::Edge,
            1,
            false,
            embedding_tag_value);
        MeshAttributeHandle<long> todo_tag_handle =
            m.register_attribute<long>("todo_tag", wmtk::PrimitiveType::Edge, 1);
        Accessor<long> acc_edge_tag = m.create_accessor(edge_tag_handle);
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 5;
        Accessor<long> acc_todo_tag = m.create_accessor(todo_tag_handle);
        acc_todo_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 1;
        OperationSettings<tet_mesh::EdgeSplitWithTags> settings;
        settings.pos_handle = pos_handle;
        settings.edge_tag_handle = edge_tag_handle;
        settings.vertex_tag_handle = vertex_tag_handle;
        settings.split_todo_handle = todo_tag_handle;
        settings.initialize_invariants(m);
        CHECK(settings.are_invariants_initialized());
        tet_mesh::EdgeSplitWithTags op(m, m.edge_tuple_between_v1_v2(1, 2, 0), settings);
        CHECK(op.name().compare("tet_mesh_split_edge_with_tags") == 0);
        CHECK(op());
        CHECK(op.return_tuple().same_ids(op.new_vertex()));
        CHECK(m.id(op.return_tuple(), PrimitiveType::Vertex) == 1);
        CHECK(acc_edge_tag.scalar_attribute(op.return_tuple()) == 5);
        CHECK(
            acc_edge_tag.scalar_attribute(m.switch_edge(m.switch_face(m.switch_tetrahedron(
                m.switch_face(m.switch_edge(m.switch_vertex(op.return_tuple()))))))) == 5);
    }
}

TEST_CASE("tet_split_with_tags", "[operation][split][3d]")
{
    using namespace operations;

    const long embedding_tag_value = 0;
    const long input_tag_value = 1;
    const long split_tag_value = 2;
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
        MeshAttributeHandle<double> pos_handle =
            m.get_attribute_handle<double>("vertices", wmtk::PrimitiveType::Vertex);
        MeshAttributeHandle<long> vertex_tag_handle = m.register_attribute<long>(
            "vertex_tag",
            wmtk::PrimitiveType::Vertex,
            1,
            false,
            embedding_tag_value);
        MeshAttributeHandle<long> edge_tag_handle = m.register_attribute<long>(
            "edge_tag",
            wmtk::PrimitiveType::Edge,
            1,
            false,
            embedding_tag_value);
        MeshAttributeHandle<long> todo_tag_handle =
            m.register_attribute<long>("todo_tag", wmtk::PrimitiveType::Edge, 1);
        Accessor<long> acc_edge_tag = m.create_accessor(edge_tag_handle);
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 5;
        Accessor<long> acc_todo_tag = m.create_accessor(todo_tag_handle);
        acc_todo_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 1;
        OperationSettings<tet_mesh::TetSplitWithTags> settings;
        settings.pos_handle = pos_handle;
        settings.edge_tag_handle = edge_tag_handle;
        settings.vertex_tag_handle = vertex_tag_handle;
        settings.split_tet_todo_handle = todo_tag_handle;
        settings.split_vertex_tag_value = split_tag_value;
        settings.initialize_invariants(m);
        CHECK(settings.are_invariants_initialized());
        tet_mesh::TetSplitWithTags op(m, m.edge_tuple_between_v1_v2(1, 2, 0), settings);
        CHECK(op.name().compare("tet_mesh_split_tet_with_tags") == 0);
        CHECK(op());
        CHECK(op.new_vertex().same_ids(op.return_tuple()));
        CHECK(m.id(op.return_tuple(), PrimitiveType::Vertex) == 1);
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
        MeshAttributeHandle<double> pos_handle =
            m.get_attribute_handle<double>("vertices", wmtk::PrimitiveType::Vertex);
        MeshAttributeHandle<long> vertex_tag_handle = m.register_attribute<long>(
            "vertex_tag",
            wmtk::PrimitiveType::Vertex,
            1,
            false,
            embedding_tag_value);
        MeshAttributeHandle<long> edge_tag_handle = m.register_attribute<long>(
            "edge_tag",
            wmtk::PrimitiveType::Edge,
            1,
            false,
            embedding_tag_value);
        MeshAttributeHandle<long> todo_tag_handle =
            m.register_attribute<long>("todo_tag", wmtk::PrimitiveType::Tetrahedron, 1);
        Accessor<long> acc_todo_tag = m.create_accessor(todo_tag_handle);
        acc_todo_tag.scalar_attribute(m.get_all(PrimitiveType::Tetrahedron)[0]) = 1;
        acc_todo_tag.scalar_attribute(m.get_all(PrimitiveType::Tetrahedron)[3]) = 1;
        OperationSettings<tet_mesh::TetSplitWithTags> settings;
        settings.pos_handle = pos_handle;
        settings.edge_tag_handle = edge_tag_handle;
        settings.vertex_tag_handle = vertex_tag_handle;
        settings.split_tet_todo_handle = todo_tag_handle;
        settings.split_vertex_tag_value = split_tag_value;
        settings.initialize_invariants(m);
        CHECK(settings.are_invariants_initialized());
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(m.edge_tuple_between_v1_v2(1, 2, 0))),
                PrimitiveType::Vertex) == 3);
        tet_mesh::TetSplitWithTags op(m, m.edge_tuple_between_v1_v2(1, 2, 0), settings);
        CHECK(op());
        CHECK(acc_todo_tag.scalar_attribute(op.return_tuple()) == 0);
        CHECK(m.id(op.return_tuple(), PrimitiveType::Vertex) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(op.return_tuple())), PrimitiveType::Vertex) == 2);
        CHECK(
            m.id(
                m.switch_vertex(m.switch_edge(
                    m.switch_face(m.switch_tetrahedron(m.switch_edge(op.return_tuple()))))),
                PrimitiveType::Vertex) == 3);
        CHECK(
            acc_todo_tag.scalar_attribute(m.switch_tetrahedron(
                m.switch_face(m.switch_tetrahedron(m.switch_edge(op.return_tuple()))))) == 1);
        tet_mesh::TetSplitWithTags op1(m, op.return_tuple(), settings);
        CHECK(!op1());
    }
}