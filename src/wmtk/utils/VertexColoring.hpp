#pragma once

#include <wmtk/threading/dynamic_parallel_for.hpp>
#include <wmtk/threading/task_group.hpp>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace wmtk::utils {

/**
 * @brief Split `vids` into classes of pairwise non-adjacent vertices (a greedy distance-1
 * coloring of the subgraph they induce).
 *
 * This is what lets a pass that touches only a vertex and its one-ring -- smoothing -- run a
 * whole class in parallel with no locks: two vertices of one class share no edge, so neither
 * moves a vertex the other reads, and they share no cell, so neither writes a cell attribute the
 * other writes. Vertices outside `vids` are not moved by the pass and do not constrain the
 * coloring.
 *
 * @param vids the vertices to color, in the order the greedy pass should visit them.
 * @param vert_capacity an upper bound on every vertex id involved.
 * @param num_threads threads used to collect the adjacency (the coloring itself is serial).
 * @param neighbors `neighbors(vid, out)` appends the vertices adjacent to `vid` to `out`;
 *        duplicates and `vid` itself are tolerated. Called concurrently for different vids.
 * @return the classes, each listing its vertices in the order they appear in `vids`.
 *
 * Deterministic: the result depends on `vids` and the adjacency only, never on `num_threads`.
 */
template <typename NeighborFn>
std::vector<std::vector<size_t>> greedy_vertex_coloring(
    const std::vector<size_t>& vids,
    size_t vert_capacity,
    int num_threads,
    NeighborFn&& neighbors)
{
    constexpr uint32_t kNone = UINT32_MAX;
    std::vector<uint32_t> slot(vert_capacity, kNone);
    for (size_t i = 0; i < vids.size(); ++i) {
        slot[vids[i]] = uint32_t(i);
    }

    // Each entry is written by the one task that owns its index.
    std::vector<std::vector<uint32_t>> adjacency(vids.size());
    threading::dynamic_parallel_for(vids.size(), num_threads, 256, [&](size_t b, size_t e) {
        std::vector<size_t> nb;
        for (size_t i = b; i < e; ++i) {
            nb.clear();
            neighbors(vids[i], nb);
            for (const size_t u : nb) {
                if (u < slot.size() && slot[u] != kNone && slot[u] != i) {
                    adjacency[i].push_back(slot[u]);
                }
            }
        }
    });

    // Greedy in `vids` order: the smallest color no already-colored neighbor holds.
    std::vector<uint32_t> color(vids.size(), kNone);
    std::vector<size_t> seen_at; // seen_at[c] == i + 1: a neighbor of vertex i holds color c
    uint32_t n_colors = 0;
    for (size_t i = 0; i < vids.size(); ++i) {
        for (const uint32_t j : adjacency[i]) {
            const uint32_t c = color[j];
            if (c != kNone) {
                seen_at[c] = i + 1;
            }
        }
        uint32_t c = 0;
        while (c < n_colors && seen_at[c] == i + 1) {
            ++c;
        }
        if (c == n_colors) {
            ++n_colors;
            seen_at.push_back(0);
        }
        color[i] = c;
    }

    std::vector<std::vector<size_t>> classes(n_colors);
    for (size_t i = 0; i < vids.size(); ++i) {
        classes[color[i]].push_back(vids[i]);
    }
    return classes;
}

/**
 * @brief The chunk size for running one color class of `n` vertices on `num_threads`.
 *
 * About eight chunks per thread, and at most `max_chunk` vertices each. A class ends in a barrier,
 * and smoothing costs vary a lot from vertex to vertex (a surface vertex projects onto the
 * envelope), so a class split into only a couple of chunks per thread waits for whichever thread
 * drew the expensive ones: on triwild 191265 (~1000 vertices per class) a fixed chunk of 32 made
 * smoothing slower than the locked queue.
 */
inline size_t color_class_chunk(size_t n, int num_threads, size_t max_chunk)
{
    const size_t per = n / (8 * size_t(num_threads > 0 ? num_threads : 1));
    return per < 1 ? 1 : (per > max_chunk ? max_chunk : per);
}

/**
 * @brief Run `fn(vid)` for every vertex of `classes`, one class after another and each class in
 * parallel; returns how many calls returned true.
 *
 * The threads are started once for all the classes and meet at a barrier between two classes,
 * rather than being started once per class: task_group starts a thread per task, and a thread
 * keeps its thread_local state -- smoothing's Newton solver, say -- only for as long as it lives.
 * Started once, they live as long as the locked pass this replaces kept them. Within a class a
 * thread takes the next color_class_chunk() vertices whenever it is done with its last ones.
 *
 * `fn` runs concurrently only for vertices of one class. If it throws, the remaining vertices are
 * skipped and the first exception is rethrown once every thread has stopped.
 *
 * The barrier counts on every task, so no task starts before all of them are launched: if
 * launching one fails (a thread cannot be created), the tasks already running leave without
 * touching the barrier and the launch failure is rethrown, instead of those tasks waiting forever
 * for one that never came.
 */
template <typename F>
size_t for_each_in_classes(
    const std::vector<std::vector<size_t>>& classes,
    int num_threads,
    size_t max_chunk,
    F&& fn)
{
    const size_t nt = size_t(num_threads > 1 ? num_threads : 1);
    if (nt == 1) {
        size_t successes = 0;
        for (const auto& cls : classes) {
            for (const size_t v : cls) {
                if (fn(v)) ++successes;
            }
        }
        return successes;
    }

    // next[c]: the first vertex of class c not yet handed out.
    auto next = std::make_unique<std::atomic<size_t>[]>(classes.size());
    for (size_t c = 0; c < classes.size(); ++c) next[c].store(0, std::memory_order_relaxed);
    // A reusable barrier: the last to arrive starts the next generation.
    std::atomic<size_t> arrived{0};
    std::atomic<size_t> generation{0};
    const auto barrier = [&] {
        const size_t gen = generation.load(std::memory_order_acquire);
        if (arrived.fetch_add(1, std::memory_order_acq_rel) + 1 == nt) {
            arrived.store(0, std::memory_order_relaxed);
            generation.fetch_add(1, std::memory_order_release);
        } else {
            while (generation.load(std::memory_order_acquire) == gen) {
                std::this_thread::yield();
            }
        }
    };
    std::atomic<size_t> successes{0};
    std::atomic<bool> failed{false};
    std::exception_ptr error;
    std::mutex error_mutex;
    // The start gate: kLaunching until every task is launched, then kGo -- or kAbort if a launch
    // failed. Declared before the task_group, which waits for the tasks when it is destroyed.
    enum : int { kLaunching = 0, kGo = 1, kAbort = 2 };
    std::atomic<int> gate{kLaunching};

    threading::task_group tg;
    const auto task = [&] {
        int state;
        while ((state = gate.load(std::memory_order_acquire)) == kLaunching) {
            std::this_thread::yield();
        }
        if (state == kAbort) return;
        size_t mine = 0;
        for (size_t c = 0; c < classes.size(); ++c) {
            const std::vector<size_t>& cls = classes[c];
            const size_t chunk = color_class_chunk(cls.size(), int(nt), max_chunk);
            while (!failed.load(std::memory_order_relaxed)) {
                const size_t b = next[c].fetch_add(chunk, std::memory_order_relaxed);
                if (b >= cls.size()) break;
                const size_t e = std::min(cls.size(), b + chunk);
                try {
                    for (size_t i = b; i < e; ++i) {
                        if (fn(cls[i])) ++mine;
                    }
                } catch (...) {
                    std::lock_guard<std::mutex> lock(error_mutex);
                    if (!error) error = std::current_exception();
                    failed.store(true, std::memory_order_relaxed);
                }
            }
            barrier();
        }
        successes.fetch_add(mine, std::memory_order_relaxed);
    };
    try {
        for (size_t t = 0; t < nt; ++t) {
            tg.run(task);
        }
    } catch (...) {
        gate.store(kAbort, std::memory_order_release);
        throw;
    }
    gate.store(kGo, std::memory_order_release);
    tg.wait();
    if (error) std::rethrow_exception(error);
    return successes.load();
}

} // namespace wmtk::utils
