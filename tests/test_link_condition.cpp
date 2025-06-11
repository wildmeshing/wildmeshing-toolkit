#include <wmtk/TetMesh.h>
#include <catch2/catch_test_macros.hpp>
#include "spdlog/common.h"
#include "wmtk/utils/Logger.hpp"

using namespace wmtk;


TEST_CASE("link_condition_single_tet", "[link]")
{
    TetMesh mesh;
    mesh.init(4, {{{0, 1, 2, 3}}});
    for (auto& e : mesh.get_edges()) {
        std::vector<TetMesh::Tuple> dummy;
        REQUIRE_FALSE(mesh.collapse_edge(e, dummy));
        REQUIRE(e.is_valid(mesh));
    }
}

TEST_CASE("link_condition_double_tet", "[link]")
{
    // https://i.imgur.com/rfAAEJh.png
    // with 3 in the center and 5 on the bottom.
    // Visualize with
    // ```python
    // V = np.array(
    //       [[ 0,  0, -1.],
    //        [ 0,  0,  1],
    //        [ 1,  1,  0],
    //        [ 1, 0,  0],
    //        [ 2,  0,  0],
    //        [1,-1,0]])
    //
    // T = np.array([[0,3,2,4], [1,2,3,4], [0,1,2,3],
    //              [0,1,3,5], [1,3,5,4], [0,3,4,5]])
    // ```
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
    std::vector<TetMesh::Tuple> dummy;
    REQUIRE(mesh.collapse_edge(oppo_t, dummy));
}