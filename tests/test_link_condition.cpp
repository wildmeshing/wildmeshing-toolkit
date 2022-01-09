#include <wmtk/TetMesh.h>
#include <catch2/catch.hpp>
#include "wmtk/utils/Logger.hpp"

using namespace wmtk;


TEST_CASE("link_condition_tet", "[test_link]")
{
    TetMesh mesh;
    mesh.init(4, {{{0, 1, 2, 3}}});
    for (auto &e : mesh.get_edges()) {
      std::vector<TetMesh::Tuple> dummy;
      REQUIRE_FALSE(mesh.collapse_edge(e, dummy));
      REQUIRE(e.is_valid(mesh));
    }
}