#include <wmtk/TetMesh.h>
#include <catch2/catch.hpp>
#include "wmtk/utils/Logger.hpp"

using namespace wmtk;

bool link_condition(const TetMesh& m, const TetMesh::Tuple& tup)
{
    auto vid0 = tup.vid(m);
    return false;
};

TEST_CASE("link_condition_tet", "[test_link]")
{
    TetMesh mesh;
    mesh.init(4, {{0, 1, 2, 3}});
    for (auto &e : mesh.get_edges()) {
      REQUIRE_FALSE(link_condition(mesh, e));
    }
}