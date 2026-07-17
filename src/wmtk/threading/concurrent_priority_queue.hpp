#pragma once

#include <mutex>
#include <queue>

namespace wmtk::threading {
// ---------------------------------------------------------------------------
// concurrent_priority_queue: replaces tbb::concurrent_priority_queue.
// Max-heap by default, matching TBB. std::priority_queue + mutex.
// Non-movable (holds a mutex); constructed in place inside std::vector(count).
// ---------------------------------------------------------------------------
template <typename T, typename Compare = std::less<T>>
class concurrent_priority_queue
{
    mutable std::mutex m_mutex;
    std::priority_queue<T, std::vector<T>, Compare> m_queue;

public:
    concurrent_priority_queue() = default;
    concurrent_priority_queue(const concurrent_priority_queue&) = delete;
    concurrent_priority_queue& operator=(const concurrent_priority_queue&) = delete;

    bool try_pop(T& out)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_queue.empty()) return false;
        out = m_queue.top();
        m_queue.pop();
        return true;
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