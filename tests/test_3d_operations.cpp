#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/Accessor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/operations/OperationFactory.hpp>

#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/TetMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests_3d;

using TM = TetMesh;
using MapResult = typename Eigen::Matrix<long, Eigen::Dynamic, 1>::MapType;
using TMOE = decltype(std::declval<DEBUG_TetMesh>().get_tmoe(
    wmtk::Tuple(),
    std::declval<Accessor<long>&>()));

constexpr PrimitiveType PV = PrimitiveType::Vertex;
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
