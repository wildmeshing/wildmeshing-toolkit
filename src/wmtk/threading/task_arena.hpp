#pragma once

#include "detail/arena_concurrency.hpp"

namespace wmtk::threading {

// ---------------------------------------------------------------------------
// task_arena: replaces tbb::task_arena. It only limits concurrency; here we just
// record the requested concurrency in a thread-local that parallel_for reads.
// ---------------------------------------------------------------------------
class task_arena
{
    int m_max_concurrency;

public:
    explicit task_arena(int max_concurrency = 0)
        : m_max_concurrency(max_concurrency)
    {}

    template <typename F>
    void execute(F&& f)
    {
        int previous = detail::arena_concurrency();
        detail::arena_concurrency() = m_max_concurrency;
        try {
            f();
        } catch (...) {
            detail::arena_concurrency() = previous;
            throw;
        }
        detail::arena_concurrency() = previous;
    }
};

} // namespace wmtk::threading