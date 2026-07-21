#pragma once

#include <functional>

#include "range.hpp"

namespace wmtk::threading {

// ---------------------------------------------------------------------------
// parallel_for: replaces tbb::parallel_for.
// The thread count is an explicit argument: `num_threads >= 0` runs on that many
// threads, anything else (`num_threads` is negative) falls back to hardware_concurrency().
// ---------------------------------------------------------------------------
void parallel_for(
    const range& range,
    std::function<void(const threading::range&)>&& func,
    int num_threads = -1);

} // namespace wmtk::threading