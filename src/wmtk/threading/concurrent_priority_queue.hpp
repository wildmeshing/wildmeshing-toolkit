#pragma once

#include <mutex>
#include <wmtk/threading/sorted_run_queue.hpp>

namespace wmtk::threading {
// ---------------------------------------------------------------------------
// concurrent_priority_queue: replaces tbb::concurrent_priority_queue.
// Max-heap by default, matching TBB. sorted_run_queue + mutex.
// Non-movable (holds a mutex); constructed in place inside std::vector(count).
// ---------------------------------------------------------------------------
template <typename T, typename Compare = std::less<T>>
class concurrent_priority_queue
{
    mutable std::mutex m_mutex;
    sorted_run_queue<T, Compare> m_queue; // grows in blocks, see sorted_run_queue

public:
    concurrent_priority_queue() = default;
    concurrent_priority_queue(const concurrent_priority_queue&) = delete;
    concurrent_priority_queue& operator=(const concurrent_priority_queue&) = delete;

    bool try_pop(T& out)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.try_pop(out);
    }

    void push(const T& v)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(v);
    }

    template <typename... Args>
    void emplace(Args&&... args)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.emplace(std::forward<Args>(args)...);
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }
};

} // namespace wmtk::threading