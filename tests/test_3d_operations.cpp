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
        for (int i = 0; i < ids_to_delete[0].size(); i++) {
            std::cout << ids_to_delete[0][i] << std::endl;
        }
        std::cout << "edge: " << std::endl;
        for (int i = 0; i < ids_to_delete[1].size(); i++) {
            std::cout << ids_to_delete[1][i] << std::endl;
        }
        std::cout << "face: " << std::endl;
        for (int i = 0; i < ids_to_delete[2].size(); i++) {
            std::cout << ids_to_delete[2][i] << std::endl;
        }
        std::cout << "tet: " << std::endl;
        for (int i = 0; i < ids_to_delete[3].size(); i++) {
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

        std::cout << "face: " << std::endl;
        for (int i = 0; i < ids_to_delete[2].size(); i++) {
            std::cout << ids_to_delete[2][i] << std::endl;
        }

        REQUIRE(ids_to_delete[0].size() == 1);
        REQUIRE(ids_to_delete[1].size() == 3);
        REQUIRE(ids_to_delete[2].size() == 3);
        REQUIRE(ids_to_delete[3].size() == 1);
    }
}