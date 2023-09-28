#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/Accessor.hpp>
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;

using EM = EdgeMesh;
using MapResult = typename Eigen::Matrix<long, Eigen::Dynamic, 1>::MapType;
using EMOE = decltype(std::declval<DEBUG_EdgeMesh>().get_emoe(
    wmtk::Tuple(),
    std::declval<Accessor<long>&>()));

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;

TEST_CASE("simplices_to_delete_for_split_1D", "[operations][1D]")
{
    SECTION("single line")
    {
        DEBUG_EdgeMesh m = single_line();
        REQUIRE(m.is_connectivity_valid());

        const long edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        REQUIRE(m.is_valid_slow(edge));
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_emoe(edge, hash_accessor);

        executor.split_edge();
        const auto& ids_to_delete = executor.simplex_ids_to_delete;
        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);
    }

    SECTION("self loop")
    {
        DEBUG_EdgeMesh m = self_loop();
        REQUIRE(m.is_connectivity_valid());

        const long edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        REQUIRE(m.is_valid_slow(edge));
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_emoe(edge, hash_accessor);

        executor.split_edge();
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

        const long edge_id = 2;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        REQUIRE(m.is_valid_slow(edge));
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_emoe(edge, hash_accessor);

        executor.collapse_edge();
        const auto& ids_to_delete = executor.simplex_ids_to_delete;
        REQUIRE(ids_to_delete[0].size() == 1);
        REQUIRE(ids_to_delete[0][0] == 3);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);
    }

    SECTION("two_line_loop")
    {
        DEBUG_EdgeMesh m = two_line_loop();
        REQUIRE(m.is_connectivity_valid());

        const long edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        REQUIRE(m.is_valid_slow(edge));
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_emoe(edge, hash_accessor);

        executor.collapse_edge();
        const auto& ids_to_delete = executor.simplex_ids_to_delete;
        REQUIRE(ids_to_delete[0].size() == 1);
        REQUIRE(ids_to_delete[0][0] == 1);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);
    }
}
