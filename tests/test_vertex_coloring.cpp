#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/VertexColoring.hpp>

#include <algorithm>
#include <atomic>
#include <memory>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace wmtk;

namespace {

/// A random symmetric graph on n vertices, about `degree` neighbours each.
std::vector<std::vector<size_t>> random_graph(const size_t n, const size_t degree, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<size_t> pick(0, n - 1);
    std::vector<std::vector<size_t>> adj(n);
    for (size_t v = 0; v < n; ++v) {
        for (size_t k = 0; k < degree / 2; ++k) {
            const size_t u = pick(rng);
            if (u == v) continue;
            adj[v].push_back(u);
            adj[u].push_back(v);
        }
    }
    return adj;
}

} // namespace

TEST_CASE("greedy_vertex_coloring", "[threading][coloring]")
{
    const size_t n = 5000;
    const auto adj = random_graph(n, 12, 3);
    // Every other vertex, shuffled: the uncolored ones must not constrain the coloring.
    std::vector<size_t> vids;
    for (size_t v = 0; v < n; v += 2) vids.push_back(v);
    std::shuffle(vids.begin(), vids.end(), std::mt19937(5));
    const auto neighbors = [&adj](size_t v, std::vector<size_t>& out) {
        out.insert(out.end(), adj[v].begin(), adj[v].end());
        out.push_back(v); // itself and duplicates are tolerated
        if (!adj[v].empty()) out.push_back(adj[v].front());
    };

    const auto classes = utils::greedy_vertex_coloring(vids, n, 4, neighbors);

    SECTION("every vertex is in exactly one class, in vids order")
    {
        std::vector<int> seen(n, 0);
        std::vector<size_t> position(n, 0);
        for (size_t i = 0; i < vids.size(); ++i) position[vids[i]] = i;
        for (const auto& cls : classes) {
            REQUIRE_FALSE(cls.empty());
            for (size_t k = 0; k < cls.size(); ++k) {
                ++seen[cls[k]];
                if (k > 0) CHECK(position[cls[k - 1]] < position[cls[k]]);
            }
        }
        for (size_t v = 0; v < n; ++v) CHECK(seen[v] == (v % 2 == 0 ? 1 : 0));
    }

    SECTION("no two vertices of a class are adjacent")
    {
        std::vector<int> color(n, -1);
        for (size_t c = 0; c < classes.size(); ++c) {
            for (const size_t v : classes[c]) color[v] = int(c);
        }
        size_t conflicts = 0;
        for (const size_t v : vids) {
            for (const size_t u : adj[v]) {
                if (u != v && color[u] == color[v]) ++conflicts;
            }
        }
        CHECK(conflicts == 0);
    }

    SECTION("the classes do not depend on the number of threads")
    {
        for (const int threads : {0, 1, 3, 16}) {
            CHECK(utils::greedy_vertex_coloring(vids, n, threads, neighbors) == classes);
        }
    }
}

TEST_CASE("for_each_in_classes", "[threading][coloring]")
{
    // Four classes of different sizes, one of them empty.
    std::vector<std::vector<size_t>> classes(4);
    size_t next_id = 0;
    const std::vector<size_t> sizes = {1000, 0, 7, 300};
    for (size_t c = 0; c < sizes.size(); ++c) {
        for (size_t k = 0; k < sizes[c]; ++k) classes[c].push_back(next_id++);
    }
    const size_t n = next_id;
    std::vector<size_t> class_of(n);
    for (size_t c = 0; c < classes.size(); ++c) {
        for (const size_t v : classes[c]) class_of[v] = c;
    }

    SECTION("each vertex once, class after class")
    {
        for (const int threads : {0, 1, 4, 16}) {
            CAPTURE(threads);
            auto visits = std::make_unique<std::atomic<int>[]>(n);
            auto done = std::make_unique<std::atomic<size_t>[]>(classes.size());
            for (size_t v = 0; v < n; ++v) visits[v] = 0;
            for (size_t c = 0; c < classes.size(); ++c) done[c] = 0;
            std::atomic<size_t> out_of_order{0};
            const size_t successes = utils::for_each_in_classes(classes, threads, 8, [&](size_t v) {
                const size_t c = class_of[v];
                // Every earlier class has finished before any vertex of this one starts.
                for (size_t d = 0; d < c; ++d) {
                    if (done[d].load() != classes[d].size()) ++out_of_order;
                }
                ++visits[v];
                ++done[c];
                return v % 3 == 0;
            });
            for (size_t v = 0; v < n; ++v) CHECK(visits[v].load() == 1);
            CHECK(out_of_order.load() == 0);
            size_t expected = 0;
            for (size_t v = 0; v < n; ++v) expected += v % 3 == 0;
            CHECK(successes == expected);
        }
    }

    SECTION("an exception in one call is rethrown once every thread has stopped")
    {
        for (const int threads : {1, 4}) {
            std::atomic<size_t> calls{0};
            try {
                utils::for_each_in_classes(classes, threads, 8, [&](size_t v) {
                    ++calls;
                    if (v == 500) throw std::runtime_error("vertex 500");
                    return true;
                });
                FAIL("no exception");
            } catch (const std::runtime_error& e) {
                CHECK(std::string(e.what()) == "vertex 500");
            }
            CHECK(calls.load() < n); // the remaining vertices were skipped
        }
    }

    SECTION("no work")
    {
        const std::vector<std::vector<size_t>> none(3);
        size_t calls = 0;
        CHECK(utils::for_each_in_classes(none, 8, 8, [&](size_t) { return ++calls > 0; }) == 0);
        CHECK(utils::for_each_in_classes({}, 8, 8, [&](size_t) { return ++calls > 0; }) == 0);
        CHECK(calls == 0);
    }
}
