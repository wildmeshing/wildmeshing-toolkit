#pragma once

#include <queue>
#include <utility>
#include <vector>

namespace wmtk::threading {
// ---------------------------------------------------------------------------
// serial_priority_queue: the same (used) interface as concurrent_priority_queue,
// minus the mutex, for queues that only ever have one thread touching them.
//
// The scheduler gives every task its own queue and only ever pops from, and pushes
// renewed operations back into, that one queue; the single queue that genuinely
// crosses threads is the overflow queue drained after the barrier. Paying for a
// lock/unlock pair on every pop and every renewal of a thread-private heap is pure
// overhead, so the private ones use this and only the shared one stays concurrent.
//
// Both are std::priority_queue underneath with the same comparator, so swapping one
// for the other cannot change pop order.
// ---------------------------------------------------------------------------
template <typename T, typename Compare = std::less<T>>
class serial_priority_queue
{
    std::priority_queue<T, std::vector<T>, Compare> m_queue;

public:
    bool try_pop(T& out)
    {
        if (m_queue.empty()) {
            return false;
        }
        out = m_queue.top();
        m_queue.pop();
        return true;
    }

    void push(const T& v) { m_queue.push(v); }

    template <typename... Args>
    void emplace(Args&&... args)
    {
        m_queue.emplace(std::forward<Args>(args)...);
    }

    std::size_t size() const { return m_queue.size(); }
    bool empty() const { return m_queue.empty(); }
};

} // namespace wmtk::threading
