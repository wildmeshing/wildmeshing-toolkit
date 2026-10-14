#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/SlotPool.hpp>
#include <wmtk/threading/task_group.hpp>

#include <algorithm>
#include <array>
#include <vector>

using namespace wmtk;

/**
 * Slot allocation is the one place the mesh hands out fresh storage while parallel operations
 * are running, and it does so without a lock: `request_tri_slots` / `request_tet_slots` and
 * `request_vert_slots` bump an atomic counter with a CAS loop and refuse (INVALID_SLOT)
 * rather than
 * resize, because resizing would move the connectivity out from under concurrent readers.
 *
 * The invariants below are what callers actually depend on -- every split and swap asks for its
 * slots up front and aborts before mutating if the answer is INVALID_SLOT -- and what any
 * future growth
 * strategy has to keep true.
 *
 * TriMesh and TetMesh implement the same contract under different names, so each test is written
 * once against the traits below and instantiated for both.
 */
namespace {

struct TriTraits
{
    using Mesh = TriMesh;

    /// A strip of `n_cells` cells. The connectivity only has to be valid enough for init().
    /// Built in place: the meshes hold atomic counters, so they are neither copyable nor movable.
    static void init_strip(Mesh& m, size_t n_cells, double factor)
    {
        std::vector<std::array<size_t, 3>> tris;
        tris.reserve(n_cells);
        for (size_t i = 0; i < n_cells; ++i) {
            tris.push_back({{i, i + 1, i + 2}});
        }
        m.set_preallocation_factor(factor);
        m.init(n_cells + 2, tris);
    }

    static size_t request_cells(Mesh& m, size_t n) { return m.request_tri_slots(n); }
    static size_t cell_capacity(const Mesh& m) { return m.tri_capacity(); }
};

struct TetTraits
{
    using Mesh = TetMesh;

    static void init_strip(Mesh& m, size_t n_cells, double factor)
    {
        std::vector<std::array<size_t, 4>> tets;
        tets.reserve(n_cells);
        for (size_t i = 0; i < n_cells; ++i) {
            tets.push_back({{i, i + 1, i + 2, i + 3}});
        }
        m.set_preallocation_factor(factor);
        m.init(n_cells + 3, tets);
    }

    static size_t request_cells(Mesh& m, size_t n) { return m.request_tet_slots(n); }
    static size_t cell_capacity(const Mesh& m) { return m.tet_capacity(); }
};

/// Spare cell slots in a freshly built mesh, measured by draining one at a time.
template <class Traits>
size_t cell_headroom(size_t n_cells, double factor)
{
    typename Traits::Mesh m;
    Traits::init_strip(m, n_cells, factor);
    size_t n = 0;
    while (Traits::request_cells(m, 1) != INVALID_SLOT) {
        ++n;
    }
    return n;
}

} // namespace

TEST_CASE("SlotPool never hands out a slot past its capacity", "[slots]")
{
    // The meshes expose only their live count -- `tri_capacity()` and friends return
    // SlotPool::live(), not the storage size -- so `live() <= capacity()`, the invariant the
    // pool exists to enforce, cannot be observed through them. It can be observed here.
    constexpr size_t cap = 40;

    SECTION("draining one at a time stops exactly at capacity")
    {
        SlotPool<int> pool;
        pool.resize(cap);
        REQUIRE(pool.capacity() == cap);
        REQUIRE(pool.live() == 0);

        size_t handed = 0;
        for (;;) {
            const size_t first = pool.request(1);
            if (first == INVALID_SLOT) {
                break;
            }
            REQUIRE(first < pool.capacity()); // never past the end of the storage
            REQUIRE(pool.live() <= pool.capacity());
            ++handed;
            REQUIRE(handed <= cap); // guards against an unbounded loop if the check is broken
        }

        REQUIRE(handed == cap);
        REQUIRE(pool.live() == cap);
    }

    SECTION("a block is served only when it fits entirely")
    {
        SlotPool<int> pool;
        pool.resize(cap);

        // Exactly the whole pool fits...
        REQUIRE(pool.request(cap) == 0);
        REQUIRE(pool.live() == cap);
        REQUIRE(pool.request(1) == INVALID_SLOT);

        // ...but one more than the whole pool never does, from a fresh pool.
        SlotPool<int> tight;
        tight.resize(cap);
        REQUIRE(tight.request(cap + 1) == INVALID_SLOT);
        REQUIRE(tight.live() == 0);

        // And at the boundary: with `cap - 1` taken, a 2-block is refused and a 1-block fits.
        SlotPool<int> edge;
        edge.resize(cap);
        REQUIRE(edge.request(cap - 1) == 0);
        REQUIRE(edge.request(2) == INVALID_SLOT);
        REQUIRE(edge.live() == cap - 1);
        REQUIRE(edge.request(1) == cap - 1);
        REQUIRE(edge.live() == cap);
        REQUIRE(edge.request(1) == INVALID_SLOT);
    }

    SECTION("an empty pool refuses everything except a zero-sized request")
    {
        SlotPool<int> pool;
        REQUIRE(pool.capacity() == 0);
        REQUIRE(pool.request(1) == INVALID_SLOT);
        REQUIRE(pool.request(0) == 0); // reports live(), consumes nothing
        REQUIRE(pool.live() == 0);
    }
}

TEMPLATE_TEST_CASE(
    "cell slot requests refuse past the preallocated capacity",
    "[slots]",
    TriTraits,
    TetTraits)
{
    using Traits = TestType;
    using Mesh = typename Traits::Mesh;

    constexpr size_t n_cells = 100;
    constexpr double factor = 2.0; // above the 64-slot floor, so the factor is what decides

    Mesh m;
    Traits::init_strip(m, n_cells, factor);

    SECTION("slots are handed out contiguously, then refused")
    {
        const size_t live = Traits::cell_capacity(m);

        size_t expected = live;
        size_t got = 0;
        while ((got = Traits::request_cells(m, 1)) != INVALID_SLOT) {
            // Each success is the next slot, and the counter advances by exactly one.
            REQUIRE(got == expected);
            REQUIRE(Traits::cell_capacity(m) == expected + 1);
            ++expected;
        }

        REQUIRE(got == INVALID_SLOT);
        REQUIRE(expected > live); // there was real headroom to consume
    }

    SECTION("a refused request leaves the counter untouched")
    {
        while (Traits::request_cells(m, 1) != INVALID_SLOT);

        // Exhausted. Further requests must keep failing without advancing the counter --
        // a leak here would push the capacity past the storage and make later iteration
        // read out of bounds.
        const size_t at_exhaustion = Traits::cell_capacity(m);
        for (size_t n : {size_t(1), size_t(2), size_t(64)}) {
            REQUIRE(Traits::request_cells(m, n) == INVALID_SLOT);
            REQUIRE(Traits::cell_capacity(m) == at_exhaustion);
        }
    }

    SECTION("a block that does not fit is refused whole, and the remainder stays available")
    {
        const size_t headroom = cell_headroom<Traits>(n_cells, factor);
        REQUIRE(headroom > 4); // otherwise this section proves nothing

        const size_t live = Traits::cell_capacity(m);

        // Consume all but three slots in one block.
        const size_t block = Traits::request_cells(m, headroom - 3);
        REQUIRE(block == live);
        REQUIRE(Traits::cell_capacity(m) == live + headroom - 3);

        // Asking for more than remains must not partially serve the request...
        const size_t before = Traits::cell_capacity(m);
        REQUIRE(Traits::request_cells(m, 5) == INVALID_SLOT);
        REQUIRE(Traits::cell_capacity(m) == before);

        // ...and must not have consumed the three that were left.
        const size_t tail = Traits::request_cells(m, 3);
        REQUIRE(tail == before);
        REQUIRE(Traits::request_cells(m, 1) == INVALID_SLOT);
    }

    SECTION("a zero-sized request reports the counter without consuming anything")
    {
        const size_t before = Traits::cell_capacity(m);
        REQUIRE(Traits::request_cells(m, 0) == before);
        REQUIRE(Traits::cell_capacity(m) == before);
    }
}

TEMPLATE_TEST_CASE(
    "vertex slot requests refuse past the preallocated capacity",
    "[slots]",
    TriTraits,
    TetTraits)
{
    using Traits = TestType;
    using Mesh = typename Traits::Mesh;

    constexpr size_t n_cells = 100;
    constexpr double factor = 2.0;

    Mesh m;
    Traits::init_strip(m, n_cells, factor);
    const size_t live = m.vert_capacity();

    size_t expected = live;
    size_t got = 0;
    while ((got = m.request_vert_slots(1)) != INVALID_SLOT) {
        REQUIRE(got == expected);
        ++expected;
    }

    REQUIRE(got == INVALID_SLOT);
    REQUIRE(expected > live);

    const size_t at_exhaustion = m.vert_capacity();
    REQUIRE(m.request_vert_slots(1) == INVALID_SLOT);
    REQUIRE(m.vert_capacity() == at_exhaustion);
}

TEMPLATE_TEST_CASE(
    "concurrent slot requests never hand out the same slot twice",
    "[slots][threading]",
    TriTraits,
    TetTraits)
{
    using Traits = TestType;
    using Mesh = typename Traits::Mesh;

    constexpr size_t n_cells = 500;
    constexpr double factor = 4.0;
    constexpr int n_threads = 8;

    // `block_sizes` drives what each thread asks for. The uniform case is the common one; the
    // mixed case matters because a thread whose block does not fit must not consume the slots a
    // smaller concurrent request could still have used.
    const std::vector<size_t> block_sizes = GENERATE(
        std::vector<size_t>{1, 1, 1, 1, 1, 1, 1, 1},
        std::vector<size_t>{1, 2, 3, 4, 1, 2, 3, 4});

    Mesh m;
    Traits::init_strip(m, n_cells, factor);
    const size_t live = Traits::cell_capacity(m);

    std::vector<std::vector<size_t>> per_thread(n_threads);

    {
        threading::task_group tg;
        for (int t = 0; t < n_threads; ++t) {
            tg.run([&, t]() {
                const size_t n = block_sizes[size_t(t)];
                for (;;) {
                    const size_t first = Traits::request_cells(m, n);
                    if (first == INVALID_SLOT) {
                        break;
                    }
                    for (size_t k = 0; k < n; ++k) {
                        per_thread[size_t(t)].push_back(first + k);
                    }
                }
            });
        }
        tg.wait();
    }

    std::vector<size_t> all;
    for (const auto& v : per_thread) {
        all.insert(all.end(), v.begin(), v.end());
    }
    std::sort(all.begin(), all.end());

    // No duplicates: two operations handed the same slot would silently share a cell.
    REQUIRE(std::adjacent_find(all.begin(), all.end()) == all.end());

    // The handed-out slots tile [live, final) exactly -- no duplicates above, and no gaps here.
    // A gap would mean a failed request advanced the counter and leaked a slot that no caller
    // owns but that iteration would still walk.
    const size_t final_size = Traits::cell_capacity(m);
    REQUIRE(all.size() == final_size - live);
    if (!all.empty()) {
        REQUIRE(all.front() == live);
        REQUIRE(all.back() == final_size - 1);
    }

    // Every thread saw the pool run dry, and nothing was handed out beyond it.
    REQUIRE(Traits::request_cells(m, 1) == INVALID_SLOT);
    REQUIRE(Traits::cell_capacity(m) == final_size);
}
