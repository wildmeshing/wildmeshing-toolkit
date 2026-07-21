#pragma once

#include <deque>
#include <mutex>

namespace wmtk::threading {
// ---------------------------------------------------------------------------
// concurrent_queue: replaces tbb::concurrent_queue (FIFO). std::deque + mutex.
// ---------------------------------------------------------------------------
template <typename T>
class concurrent_queue
{
    mutable std::mutex m_mutex;
    std::deque<T> m_queue;

public:
    concurrent_queue() = default;
    concurrent_queue(const concurrent_queue&) = delete;
    concurrent_queue& operator=(const concurrent_queue&) = delete;

    bool try_pop(T& out)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_queue.empty()) {
            return false;
        }
        out = std::move(m_queue.front());
        m_queue.pop_front();
        return true;
    }
    void push(const T& v)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push_back(v);
    }
    template <typename... Args>
    void emplace(Args&&... args)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.emplace_back(std::forward<Args>(args)...);
    }

    // std::size_t unsafe_size() const { return m_queue.size(); }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }
};

} // namespace wmtk::threading