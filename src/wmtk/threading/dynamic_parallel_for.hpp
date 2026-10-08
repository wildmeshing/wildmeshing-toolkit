#pragma once

#include <wmtk/threading/task_group.hpp>

#include <algorithm>
#include <atomic>
#include <cstddef>

namespace wmtk::threading {

/**
 * @brief Run `fn(begin, end)` over [0, n) in chunks of `chunk` items, handed out on demand to
 * `num_threads` tasks.
 *
 * Unlike parallel_for, which gives each thread one fixed slice, a task here takes the next chunk
 * whenever it finishes one, so items of very different cost -- the one-ring gathers of vertices
 * of very different valence, say -- still keep every thread busy.
 *
 * Which task runs which chunk is arbitrary. Callers that need a result independent of the
 * number of threads must make the items independent of each other.
 *
 * With one task (num_threads <= 1, or n no larger than one chunk) `fn` runs on the calling
 * thread.
 */
template <typename F>
void dynamic_parallel_for(size_t n, int num_threads, size_t chunk, F&& fn)
{
    if (n == 0) {
        return;
    }
    chunk = std::max<size_t>(1, chunk);
    const size_t n_chunks = (n + chunk - 1) / chunk;
    const size_t n_tasks = std::min<size_t>(size_t(std::max(1, num_threads)), n_chunks);
    if (n_tasks <= 1) {
        fn(size_t(0), n);
        return;
    }

    std::atomic<size_t> next{0};
    task_group tg;
    for (size_t t = 0; t < n_tasks; ++t) {
        tg.run([&next, &fn, n, chunk]() {
            for (;;) {
                const size_t begin = next.fetch_add(chunk, std::memory_order_relaxed);
                if (begin >= n) {
                    break;
                }
                fn(begin, std::min(n, begin + chunk));
            }
        });
    }
    tg.wait();
}

} // namespace wmtk::threading
