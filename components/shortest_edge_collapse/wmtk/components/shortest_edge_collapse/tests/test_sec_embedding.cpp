// count_crossings(): the exact predicate it decides with, the box tree it finds the candidates
// with, and the counts.

#include <wmtk/components/shortest_edge_collapse/Embedding.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <random>
#include <set>
#include <vector>

using namespace wmtk;
using namespace components::shortest_edge_collapse;

namespace {

using V3 = Eigen::Vector3d;
using Ids = std::array<size_t, 3>;
using Tri = std::array<V3, 3>;

/// The predicate, checked to be the same for the pair either way round and under a rotation of
/// either triangle's corners.
bool meet(const Ids& ia, const Tri& A, const Ids& ib, const Tri& B)
{
    const bool m = triangles_meet_outside_shared(ia, A, ib, B);
    CHECK(triangles_meet_outside_shared(ib, B, ia, A) == m);
    const Ids ra = {{ia[1], ia[2], ia[0]}};
    const Tri rA = {{A[1], A[2], A[0]}};
    const Ids rb = {{ib[2], ib[0], ib[1]}};
    const Tri rB = {{B[2], B[0], B[1]}};
    CHECK(triangles_meet_outside_shared(ra, rA, rb, rB) == m);
    return m;
}

const Ids a_ids = {{0, 1, 2}};
const Tri A = {{V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0)}}; // in z = 0
const Ids b_ids = {{3, 4, 5}}; // no corner shared

} // namespace

TEST_CASE("embedding-predicate-no-shared-vertex", "[test_sec][embedding]")
{
    // Apart, in parallel planes and side by side in one.
    CHECK(!meet(a_ids, A, b_ids, {{V3(0, 0, 1), V3(1, 0, 1), V3(0, 1, 1)}}));
    CHECK(!meet(a_ids, A, b_ids, {{V3(1, 1, 0), V3(2, 1, 0), V3(1, 2, 0)}}));
    // An edge through the interior.
    CHECK(meet(a_ids, A, b_ids, {{V3(0.25, 0.25, -1), V3(0.25, 0.25, 1), V3(-1, -1, 0)}}));
    // Touching at one point: a corner on the interior, a corner on an edge, an edge across an
    // edge.
    CHECK(meet(a_ids, A, b_ids, {{V3(0.25, 0.25, 0), V3(0.25, 0.25, 1), V3(1, 1, 1)}}));
    CHECK(meet(a_ids, A, b_ids, {{V3(0.5, 0, 0), V3(0.5, -1, 1), V3(0.5, 1, 1)}}));
    CHECK(meet(a_ids, A, b_ids, {{V3(0.5, -1, -1), V3(0.5, 1, 1), V3(0.5, -1, 1)}}));
    // Coplanar: overlapping, touching at a point, one inside the other, along a common segment
    // of two edges.
    CHECK(meet(a_ids, A, b_ids, {{V3(0.2, 0.2, 0), V3(1.2, 0.2, 0), V3(0.2, 1.2, 0)}}));
    CHECK(meet(a_ids, A, b_ids, {{V3(0.5, 0.5, 0), V3(1.5, 0.5, 0), V3(0.5, 1.5, 0)}}));
    CHECK(meet(a_ids, A, b_ids, {{V3(0.1, 0.1, 0), V3(0.2, 0.1, 0), V3(0.1, 0.2, 0)}}));
    CHECK(meet(a_ids, A, b_ids, {{V3(0.5, 0, 0), V3(1.5, 0, 0), V3(1, -1, 0)}}));
    // Coplanar in a plane through the z axis, which projects onto the (x, y) plane as a segment.
    const Tri X = {{V3(0, 0, 0), V3(0, 1, 0), V3(0, 0, 1)}};
    CHECK(meet(a_ids, X, b_ids, {{V3(0, 0.2, 0.2), V3(0, 1.2, 0.2), V3(0, 0.2, 1.2)}}));
    CHECK(!meet(a_ids, X, b_ids, {{V3(0, 1, 1), V3(0, 2, 1), V3(0, 1, 2)}}));
}

TEST_CASE("embedding-predicate-is-exact", "[test_sec][embedding]")
{
    // A corner one ulp below or above the plane z = (x + y) / 3, under the point (1, 1, 2/3),
    // which no double holds: below it, the triangle reaches through; above, it stays clear.
    const Tri T = {{V3(0, 0, 0), V3(3, 0, 1), V3(0, 3, 1)}};
    const double below = 2.0 / 3.0; // the nearest double, under 2/3
    const double above = std::nextafter(below, 1.0);
    CHECK(meet(a_ids, T, b_ids, {{V3(1, 1, below), V3(1, 1, 2), V3(2, 1, 2)}}));
    CHECK(!meet(a_ids, T, b_ids, {{V3(1, 1, above), V3(1, 1, 2), V3(2, 1, 2)}}));
    // A corner 1e-300 off a plane: off it is off it.
    CHECK(!meet(a_ids, A, b_ids, {{V3(0.25, 0.25, 1e-300), V3(0.25, 0.25, 1), V3(1, 1, 1)}}));
    CHECK(meet(a_ids, A, b_ids, {{V3(0.25, 0.25, -1e-300), V3(0.25, 0.25, 1), V3(1, 1, 1)}}));
}

TEST_CASE("embedding-predicate-shared-vertex", "[test_sec][embedding]")
{
    // Corner 0 shared, at the origin.
    const auto with_0 = [](const V3& p, const V3& q) { return Tri{{V3(0, 0, 0), p, q}}; };
    const Ids ids = {{0, 3, 4}};
    // Meeting at the vertex only: out of the plane, the opposite quadrant, opposite rays.
    CHECK(!meet(a_ids, A, ids, with_0(V3(0, 0, 1), V3(-1, 0, 1))));
    CHECK(!meet(a_ids, A, ids, with_0(V3(-1, 0, 0), V3(0, -1, 0))));
    CHECK(!meet(a_ids, A, ids, with_0(V3(-1, 0, 0), V3(-1, -1, 0))));
    // The opposite edge through the other.
    CHECK(meet(a_ids, A, ids, with_0(V3(0.5, 0.25, 1), V3(0.25, 0.5, -1))));
    // Coplanar corners that overlap, or share a ray from the vertex.
    CHECK(meet(a_ids, A, ids, with_0(V3(1, 1, 0), V3(-1, 2, 0))));
    CHECK(meet(a_ids, A, ids, with_0(V3(2, 0, 0), V3(1, -1, 0))));
    // An edge out of the vertex in the other's plane and corner, the triangle itself not
    // coplanar.
    CHECK(meet(a_ids, A, ids, with_0(V3(0.5, 0.5, 0), V3(0, 0, 1))));
    // That edge 1e-300 above the plane, or below, where the opposite edge then crosses.
    CHECK(!meet(a_ids, A, ids, with_0(V3(0.5, 0.5, 1e-300), V3(0, 0, 1))));
    CHECK(meet(a_ids, A, ids, with_0(V3(0.5, 0.5, -1e-300), V3(0, 0, 1))));
}

TEST_CASE("embedding-predicate-shared-edge", "[test_sec][embedding]")
{
    // Edge 0-1 shared, (0,0,0)-(1,0,0).
    const auto with_01 = [](const V3& p) { return Tri{{V3(0, 0, 0), V3(1, 0, 0), p}}; };
    const Ids ids = {{0, 1, 5}};
    CHECK(!meet(a_ids, A, ids, with_01(V3(0.5, 1, 1)))); // bent
    CHECK(!meet(a_ids, A, ids, with_01(V3(0.5, -1, 0)))); // coplanar, opposite sides
    CHECK(meet(a_ids, A, ids, with_01(V3(0.5, 0.5, 0)))); // coplanar, one side: a flat fold
    CHECK(!meet(a_ids, A, ids, with_01(V3(0.5, 0.5, 1e-300)))); // a hair from flat
    // The same triangle twice.
    CHECK(meet(a_ids, A, a_ids, A));
}

TEST_CASE("embedding-box-tree-finds-every-meeting-box", "[test_sec][embedding]")
{
    // Against brute force, empty boxes among them: the tree may only make the search faster,
    // never miss a box.
    std::mt19937 rng(7);
    const auto u = [&rng] { return double(rng()) / 4294967296.0; };
    const auto random_box = [&](double size) {
        const V3 c(u(), u(), u());
        return BoxTree::Box(c).extend(c + size * V3(u(), u(), u()));
    };
    std::vector<BoxTree::Box> boxes(500);
    for (size_t i = 0; i < boxes.size(); ++i) {
        boxes[i] = i % 5 == 0 ? BoxTree::Box() : random_box(0.2 * u());
    }
    const BoxTree tree(boxes);
    for (int round = 0; round < 200; ++round) {
        const BoxTree::Box q = random_box(0.1);
        std::set<size_t> found, expected;
        tree.any(q, [&](size_t j) {
            found.insert(j);
            return false; // go on
        });
        for (size_t j = 0; j < boxes.size(); ++j) {
            if (boxes[j].intersects(q)) expected.insert(j);
        }
        REQUIRE(found == expected);
    }
}

TEST_CASE("embedding-count-crossings", "[test_sec][embedding]")
{
    // Two tetrahedra, one's apex poking up through the other's base: the three sides at that
    // apex each cross the base, and nothing else meets. A flat triangle is counted apart, and
    // left out of the pairs although it lies across the base. The same with threads.
    std::vector<V3> V = {
        V3(0, 0, 0),
        V3(1, 0, 0),
        V3(0, 1, 0),
        V3(0, 0, 1), // the first, its base in z = 0
        V3(0.2, 0.2, 0.1),
        V3(-1, -1, -1),
        V3(3, -1, -1),
        V3(-1, 3, -1), // the second, its apex inside the first
        V3(-1, 0.5, 0),
        V3(1, 0.5, 0),
        V3(2, 0.5, 0)}; // collinear, across the base
    const std::vector<std::array<size_t, 3>> F = {
        {{0, 2, 1}},
        {{0, 1, 3}},
        {{1, 2, 3}},
        {{2, 0, 3}},
        {{5, 6, 7}},
        {{4, 6, 5}},
        {{4, 7, 6}},
        {{4, 5, 7}},
        {{8, 9, 10}}};
    for (const int threads : {0, 4}) {
        const Crossings c = count_crossings(V, F, threads);
        CHECK(c.pairs == 3);
        CHECK(c.degenerate == 1);
    }
    // Moved down out of the first, the apex crosses nothing.
    V[4] = V3(0.2, 0.2, -0.1);
    CHECK(count_crossings(V, F, 0).pairs == 0);
}
