#include "parallel_for.hpp"

#include <thread>

namespace wmtk::threading {

void parallel_for(
    const range& range,
    std::function<void(const threading::range&)>&& func,
    int num_threads)
{
    if (range.empty()) {
        return;
    }

    const std::size_t total = range.size();

    const unsigned hw = std::thread::hardware_concurrency();
    int requested = num_threads;
    std::size_t nthreads = requested >= 0 ? static_cast<std::size_t>(requested) : hw;
    nthreads = std::min<std::size_t>(nthreads, total);
    if (nthreads <= 1) {
        func(range);
        return;
    }

    const std::size_t chunk = (total + nthreads - 1) / nthreads;

    std::exception_ptr eptr;
    std::mutex eptr_mutex;
    auto run = [&](size_t cb, size_t ce) {
        try {
            func(threading::range(cb, ce));
        } catch (...) {
            std::lock_guard<std::mutex> lock(eptr_mutex);
            if (!eptr) {
                eptr = std::current_exception();
            }
        }
    };

    const size_t begin = range.begin();

    std::vector<std::thread> workers;
    workers.reserve(nthreads - 1);
    std::size_t offset = 0;
    for (std::size_t t = 0; t < nthreads && offset < total; ++t) {
        const std::size_t cb = offset;
        const std::size_t ce = std::min(total, offset + chunk);
        offset = ce;
        const size_t rb = begin + static_cast<size_t>(cb);
        const size_t re = begin + static_cast<size_t>(ce);
        if (t + 1 == nthreads || offset >= total) {
            // run the last chunk on the calling thread
            run(rb, re);
        } else {
            workers.emplace_back([&run, rb, re]() { run(rb, re); });
        }
    }
    for (auto& w : workers) {
        w.join();
    }

    if (eptr) {
        std::rethrow_exception(eptr);
    }
}

}