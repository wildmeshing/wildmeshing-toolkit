#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/polyfem_ops/InterfaceSelection.hpp>

#include <array>
#include <vector>

using namespace wmtk::components::polyfem_ops;

using Edges = std::vector<std::array<int64_t, 2>>;

// The 2D collision proxy's 'l' lines come out of orient_edge_loops_2d, so a loop that comes back
// wound the wrong way is a proxy whose normals point into the material. Every expectation below
// was read off the Python it mirrors (constraints._orient_edge_loops_2d) on the same input.
TEST_CASE("orient_edge_loops_2d keeps the direction the input edges agree on", "[polyfem_ops]")
{
    // A square, its four edges all wound the same way: the loop is returned unchanged, starting
    // at the minimum vertex (the deterministic seed).
    const Edges ccw{{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    CHECK(orient_edge_loops_2d(ccw) == ccw);

    // The same square wound the other way: the reconstruction always walks 0 -> 1 -> 2 -> 3, so
    // the loop is reversed to put the input's direction back.
    const Edges cw{{1, 0}, {2, 1}, {3, 2}, {0, 3}};
    const Edges cw_expected{{3, 2}, {2, 1}, {1, 0}, {0, 3}};
    CHECK(orient_edge_loops_2d(cw) == cw_expected);
}

TEST_CASE("orient_edge_loops_2d passes non-loop components through", "[polyfem_ops]")
{
    // An open chain is not a loop (its ends have degree 1), so it keeps its original order and
    // orientation. This is the common 2D case: an interface that ends on the domain boundary.
    const Edges chain{{0, 1}, {1, 2}};
    CHECK(orient_edge_loops_2d(chain) == chain);

    // Components are emitted in order of first vertex appearance, and a non-loop component's
    // edges in their original index order -- the chain (seeded at 5) before the loop.
    const Edges mixed{{5, 6}, {0, 1}, {1, 2}, {2, 3}, {3, 0}, {6, 7}};
    const Edges mixed_expected{{5, 6}, {6, 7}, {0, 1}, {1, 2}, {2, 3}, {3, 0}};
    CHECK(orient_edge_loops_2d(mixed) == mixed_expected);
}

TEST_CASE("orient_edge_loops_2d rejects a contradictory loop", "[polyfem_ops]")
{
    // Three edges one way and one the other: the same interface selected from both sides, so no
    // single orientation is correct for both bodies. Guessing here would silently flip a normal.
    const Edges contradictory{{0, 1}, {1, 2}, {2, 3}, {0, 3}};
    CHECK_THROWS(orient_edge_loops_2d(contradictory));
}
