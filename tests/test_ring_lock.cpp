#include <catch2/catch_test_macros.hpp>

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>

#include <algorithm>
#include <array>
#include <set>
#include <vector>

using namespace wmtk;

namespace {

/**
 * @brief A (w+1) x (h+1) grid of vertices triangulated into 2*w*h triangles.
 *
 * Big enough that a 3-ring around an interior vertex is a proper subset of the mesh, which is
 * what makes "the ball is closed" a real assertion rather than "everything got locked".
 */
void make_grid(TriMesh& m, const size_t w, const size_t h)
{
    const auto vid = [w](size_t i, size_t j) { return j * (w + 1) + i; };
    std::vector<std::array<size_t, 3>> tris;
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            tris.push_back({{vid(i, j), vid(i + 1, j), vid(i, j + 1)}});
            tris.push_back({{vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)}});
        }
    }
    m.init((w + 1) * (h + 1), tris);
}

/// The vertices within @p n edges of @p seeds, computed independently of the locker.
std::set<size_t> reference_ball(const TriMesh& m, const std::vector<size_t>& seeds, int n)
{
    std::set<size_t> ball(seeds.begin(), seeds.end());
    std::vector<size_t> frontier = seeds;
    for (int d = 0; d < n; ++d) {
        std::vector<size_t> next;
        for (const size_t v : frontier) {
            for (const size_t w : m.get_one_ring_vids_for_vertex_duplicate(v)) {
                if (ball.insert(w).second) {
                    next.push_back(w);
                }
            }
        }
        frontier = next;
    }
    return ball;
}

std::set<size_t> locked_set(TriMesh& m)
{
    const auto& stack = m.mutex_release_stack.local();
    return std::set<size_t>(stack.begin(), stack.end());
}

} // namespace

TEST_CASE("ring_lock_claims_the_whole_ball", "[threading][lock]")
{
    TriMesh m;
    make_grid(m, 8, 8);
    const size_t center = 4 * 9 + 4; // an interior vertex, far from every boundary

    for (int n = 0; n <= 3; ++n) {
        REQUIRE(m.try_set_vertex_mutex_n_ring(center, 0, n));
        const std::set<size_t> expected = reference_ball(m, {center}, n);
        REQUIRE(locked_set(m) == expected);
        // A proper subset of the mesh, or the assertion above is vacuous.
        REQUIRE(expected.size() < m.vert_capacity());
        m.release_vertex_mutex_in_stack();
        REQUIRE(m.mutex_release_stack.local().empty());
    }
}

TEST_CASE("ring_lock_edge_seeds_from_both_ends", "[threading][lock]")
{
    TriMesh m;
    make_grid(m, 8, 8);
    const TriMesh::Tuple e = m.tuple_from_edge(20, 0);
    const size_t v1 = e.vid(m);
    const size_t v2 = e.switch_vertex(m).vid(m);

    REQUIRE(m.try_set_edge_mutex_n_ring(e, 0, 3));
    REQUIRE(locked_set(m) == reference_ball(m, {v1, v2}, 3));
    m.release_vertex_mutex_in_stack();
}

TEST_CASE("ring_lock_expands_through_vertices_already_owned", "[threading][lock]")
{
    // The hand-written two-ring lockers this replaces skipped a vertex this thread already
    // held and therefore never looked at its neighbours, so the claimed set fell short of the
    // name. Taking the seed first and then asking for the ball around it is exactly that
    // situation, and the result must not depend on the order.
    TriMesh m;
    make_grid(m, 8, 8);
    const size_t center = 4 * 9 + 4;

    REQUIRE(m.try_set_vertex_mutex_n_ring(center, 0, 1));
    REQUIRE(m.try_set_vertex_mutex_n_ring(center, 0, 3));
    const std::set<size_t> incremental = locked_set(m);
    m.release_vertex_mutex_in_stack();

    REQUIRE(m.try_set_vertex_mutex_n_ring(center, 0, 3));
    REQUIRE(locked_set(m) == incremental);
    m.release_vertex_mutex_in_stack();
}

TEST_CASE("ring_lock_failure_unwinds_to_its_own_watermark", "[threading][lock]")
{
    TriMesh m;
    make_grid(m, 8, 8);
    const size_t a = 4 * 9 + 4;
    const size_t b = 4 * 9 + 5; // adjacent, so any ball around it meets a's

    // Thread 0 holds a one-ring.
    REQUIRE(m.try_set_vertex_mutex_n_ring(a, 0, 1));
    const std::set<size_t> held = locked_set(m);
    const size_t mark = m.mutex_release_stack.local().size();
    REQUIRE(mark > 0);

    // A second acquisition, under a different thread id, has to fail on a's mutex -- and must
    // leave the first one's locks in place rather than clearing the whole stack.
    REQUIRE_FALSE(m.try_set_vertex_mutex_n_ring(b, 1, 2));
    REQUIRE(m.mutex_release_stack.local().size() == mark);
    REQUIRE(locked_set(m) == held);

    m.release_vertex_mutex_in_stack();
    REQUIRE(m.mutex_release_stack.local().empty());

    // With nothing held, the same acquisition succeeds -- i.e. the failure above released
    // everything it had taken on the way, and left no mutex stranded.
    REQUIRE(m.try_set_vertex_mutex_n_ring(b, 1, 2));
    REQUIRE(locked_set(m) == reference_ball(m, {b}, 2));
    m.release_vertex_mutex_in_stack();
}

TEST_CASE("named_two_ring_helpers_are_not_balls", "[threading][lock]")
{
    // Pins the difference the "Ring lockers -- NOT balls" note in TriMesh.h / TetMesh.h
    // describes, so that swapping the named helpers onto lock_vertex_ball fails here rather
    // than silently costing +80% wall clock on the challenging tetwild set.
    //
    // Two facts, both load-bearing:
    //   1. the named helper claims a STRICT SUBSET of the ball -- it skips expanding through a
    //      vertex the thread already owns, and v2 is owned before v1's ring is walked;
    //   2. it nevertheless claims the complete ONE-ring of the edge, which is the operation's
    //      write set and the reason the shortfall in the second ring is not a race.
    TriMesh m;
    make_grid(m, 8, 8);
    const TriMesh::Tuple e = m.tuple_from_edge(60, 0);
    const size_t v1 = e.vid(m);
    const size_t v2 = e.switch_vertex(m).vid(m);

    REQUIRE(m.try_set_edge_mutex_two_ring(e, 0));
    const std::set<size_t> named = locked_set(m);
    m.release_vertex_mutex_in_stack();

    REQUIRE(m.try_set_edge_mutex_n_ring(e, 0, 2));
    const std::set<size_t> ball = locked_set(m);
    m.release_vertex_mutex_in_stack();

    REQUIRE(std::includes(ball.begin(), ball.end(), named.begin(), named.end()));
    REQUIRE(named.size() < ball.size());

    const std::set<size_t> one_ring = reference_ball(m, {v1, v2}, 1);
    REQUIRE(std::includes(named.begin(), named.end(), one_ring.begin(), one_ring.end()));
}

TEST_CASE("ring_lock_zero_claims_only_the_seed", "[threading][lock]")
{
    TriMesh m;
    make_grid(m, 4, 4);
    const size_t center = 2 * 5 + 2;
    REQUIRE(m.try_set_vertex_mutex_n_ring(center, 0, 0));
    REQUIRE(locked_set(m) == std::set<size_t>{center});
    m.release_vertex_mutex_in_stack();
}

TEST_CASE("ring_lock_saturates_at_the_whole_component", "[threading][lock]")
{
    // A radius past the mesh diameter must terminate and claim everything, not loop.
    TriMesh m;
    make_grid(m, 3, 3);
    REQUIRE(m.try_set_vertex_mutex_n_ring(size_t(0), 0, 50));
    REQUIRE(locked_set(m).size() == 16);
    m.release_vertex_mutex_in_stack();
}

TEST_CASE("tet_ring_lock_claims_the_whole_ball", "[threading][lock]")
{
    // One tet: every vertex is within one edge of every other, so the ball saturates at n = 1
    // and the seed alone at n = 0. Enough to check the 3D entry points are wired up.
    TetMesh m;
    m.init(4, {{{0, 1, 2, 3}}});

    REQUIRE(m.try_set_vertex_mutex_n_ring(size_t(0), 0, 0));
    REQUIRE(m.mutex_release_stack.local() == std::vector<size_t>{0});
    m.release_vertex_mutex_in_stack();

    REQUIRE(m.try_set_vertex_mutex_n_ring(size_t(0), 0, 1));
    const auto& stack = m.mutex_release_stack.local();
    REQUIRE(std::set<size_t>(stack.begin(), stack.end()) == std::set<size_t>{0, 1, 2, 3});
    m.release_vertex_mutex_in_stack();
}
