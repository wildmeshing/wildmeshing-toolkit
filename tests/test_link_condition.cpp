#include <wmtk/TetMesh.h>
#include <catch2/catch.hpp>
#include "spdlog/common.h"
#include "wmtk/utils/Logger.hpp"

using namespace wmtk;


TEST_CASE("link_condition_tet", "[link]")
{
    TetMesh mesh;
    mesh.init(4, {{{0, 1, 2, 3}}});
    for (auto& e : mesh.get_edges()) {
        std::vector<TetMesh::Tuple> dummy;
        REQUIRE_FALSE(mesh.collapse_edge(e, dummy));
        REQUIRE(e.is_valid(mesh));
    }
}

TEST_CASE("link_condition_2", "[link]")
{
    TetMesh mesh;
    mesh.init(
        6,
        {{{0, 3, 2, 4}},
         {{1, 2, 3, 4}},
         {{0, 1, 2, 3}},
         {{0, 1, 3, 5}},
         {{1, 3, 5, 4}},
         {{0, 3, 4, 5}}});
    auto tup = mesh.tuple_from_edge(3, 5);
    REQUIRE(tup.vid(mesh) == 3);

    auto oppo_t = tup.switch_vertex(mesh);
    REQUIRE(oppo_t.vid(mesh) == 5);
    // wmtk::logger().set_level(spdlog::level::trace);
    // wmtk::logger().flush_on(spdlog::level::trace);
    std::vector<TetMesh::Tuple> dummy;
    REQUIRE(mesh.collapse_edge(oppo_t, dummy));
    // exit(1);
    
}