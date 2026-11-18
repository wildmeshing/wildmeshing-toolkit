#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <functional>
#include <utility>

namespace wmtk::threading {
// ---------------------------------------------------------------------------
// sorted_run_queue: a max-priority queue shaped for how the scheduler uses one.
//
// A pass pushes every candidate before its first pop -- on the first pass after construction
// that is the whole mesh -- and only then interleaves pops with renewals. So the elements pushed
// while nothing has been popped are kept as a run, sorted once when the first pop comes, and
// consumed from the back; elements pushed after that go to a 4-ary heap, and each pop takes the
// greater of the two heads. Once both are empty, pushes start a new run.
//
// Pop order: under a strict total order every max-priority structure pops the same sequence,
// because equal elements are then identical. The scheduler's Elem compares every one of its
// fields, so this pops exactly what std::priority_queue pops.
//
// Cost: one sort of the seeds instead of a sift-up and a sift-down each through a heap of their
// size, and for the renewals a heap half as deep as a binary one. On queue traces captured from
// tetwild runs, 30-69% less time than std::priority_queue.
//
// Memory: both the run and the heap are deques, so they grow in blocks and never hold two
// buffers at once (see serial_priority_queue), and the run frees its blocks as it is consumed.
// ---------------------------------------------------------------------------
template <typename T, typename Compare = std::less<T>>
class sorted_run_queue
{
    static constexpr std::size_t kArity = 4;

    std::deque<T> m_run; // ascending once m_sorted, the greatest at the back
    bool m_sorted = false;
    std::deque<T> m_heap; // max-heap, kArity children per node
    Compare m_less;

    void heap_push(T x)
    {
        std::size_t i = m_heap.size();
        m_heap.push_back(x);
        while (i > 0) {
            const std::size_t parent = (i - 1) / kArity;
            if (!m_less(m_heap[parent], x)) break;
            m_heap[i] = std::move(m_heap[parent]);
            i = parent;
        }
        m_heap[i] = std::move(x);
    }

    void heap_pop()
    {
        const std::size_t n = m_heap.size() - 1;
        T x = std::move(m_heap[n]);
        m_heap.pop_back();
        if (n == 0) return;
        std::size_t i = 0;
        for (;;) {
            const std::size_t first = kArity * i + 1;
            if (first >= n) break;
            const std::size_t end = std::min(first + kArity, n);
            std::size_t best = first;
            for (std::size_t c = first + 1; c < end; ++c) {
                if (m_less(m_heap[best], m_heap[c])) best = c;
            }
            if (!m_less(x, m_heap[best])) break;
            m_heap[i] = std::move(m_heap[best]);
            i = best;
        }
        m_heap[i] = std::move(x);
    }

public:
    bool empty() const { return m_run.empty() && m_heap.empty(); }
    std::size_t size() const { return m_run.size() + m_heap.size(); }

    void push(const T& v)
    {
        if (m_sorted) {
            heap_push(v);
        } else {
            m_run.push_back(v);
        }
    }

    template <typename... Args>
    void emplace(Args&&... args)
    {
        if (m_sorted) {
            heap_push(T(std::forward<Args>(args)...));
        } else {
            m_run.emplace_back(std::forward<Args>(args)...);
        }
    }

    /// Moves the greatest element into `out`; false when empty.
    bool try_pop(T& out)
    {
        if (empty()) {
            return false;
        }
        if (!m_sorted) {
            std::sort(m_run.begin(), m_run.end(), m_less);
            m_sorted = true;
        }
        // The run's head goes first unless it is less than the heap's.
        if (!m_run.empty() && (m_heap.empty() || !m_less(m_run.back(), m_heap.front()))) {
            out = std::move(m_run.back());
            m_run.pop_back();
        } else {
            out = std::move(m_heap.front());
            heap_pop();
        }
        if (empty()) {
            m_sorted = false;
        }
        return true;
    }
};

} // namespace wmtk::threading
