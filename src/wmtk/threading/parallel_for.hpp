#pragma once

#include "blocked_range.hpp"
#include "container_range.hpp"
#include "detail/arena_concurrency.hpp"

namespace wmtk::threading {

// ---------------------------------------------------------------------------
// parallel_for: replaces tbb::parallel_for.
// ---------------------------------------------------------------------------
template <typename Index, typename Func>
void parallel_for(const blocked_range<Index>& range, Func&& func)
{
    const Index begin = range.begin();
    const Index end = range.end();
    if (!(begin < end)) return;

    const std::size_t total = static_cast<std::size_t>(end - begin);

    unsigned hw = std::thread::hardware_concurrency();
    if (hw == 0) hw = 1;
    int requested = detail::arena_concurrency();
    std::size_t nthreads = requested > 0 ? static_cast<std::size_t>(requested) : hw;
    nthreads = std::min<std::size_t>(nthreads, total);
    if (nthreads <= 1) {
        func(blocked_range<Index>(begin, end));
        return;
    }

    const std::size_t chunk = (total + nthreads - 1) / nthreads;

    std::exception_ptr eptr;
    std::mutex eptr_mutex;
    auto run = [&](Index cb, Index ce) {
        try {
            func(blocked_range<Index>(cb, ce));
        } catch (...) {
            std::lock_guard<std::mutex> lock(eptr_mutex);
            if (!eptr) eptr = std::current_exception();
        }
    };

    std::vector<std::thread> workers;
    workers.reserve(nthreads - 1);
    std::size_t offset = 0;
    for (std::size_t t = 0; t < nthreads && offset < total; ++t) {
        const std::size_t cb = offset;
        const std::size_t ce = std::min(total, offset + chunk);
        offset = ce;
        const Index rb = begin + static_cast<Index>(cb);
        const Index re = begin + static_cast<Index>(ce);
        if (t + 1 == nthreads || offset >= total) {
            // run the last chunk on the calling thread
            run(rb, re);
        } else {
            workers.emplace_back([&run, rb, re]() { run(rb, re); });
        }
    }
    for (auto& w : workers) w.join();

    if (eptr) std::rethrow_exception(eptr);
}

// Serial overload used for iterating a concurrent_map via .range().
template <typename It, typename Func>
void parallel_for(container_range<It> range, Func&& func)
{
    func(range);
}

} // namespace wmtk::threading