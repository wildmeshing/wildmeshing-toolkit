#pragma once

#include <vector>

namespace wmtk::threading {

// indexed_collector: when the loop already knows a unique slot for each result --
// the edge collectors know it as `tup.eid(m)`, which is what the loop tests to decide
// which triangle owns an edge -- each thread writes to a slot no other thread can
// touch, so no synchronisation is required at all. compact() then returns the results
// in slot order, which does not depend on how the range was scheduled: unlike a
// mutex-guarded append, whose order is whichever thread got the lock first, this is
// reproducible run to run.
template <typename T>
class indexed_collector
{
    std::vector<T> m_slots;
    std::vector<char> m_filled;

public:
    explicit indexed_collector(std::size_t n)
        : m_slots(n)
        , m_filled(n, 0)
    {}

    /**
     * @brief Set the value at index i to v.
     * This function can be called concurrently as long as callers pass distinct `i`.
     *
     * @param i The index to set.
     * @param v The value to set.
     */
    void set(std::size_t i, const T& v)
    {
        m_slots[i] = v;
        m_filled[i] = 1;
    }

    std::vector<T> compact() const
    {
        std::vector<T> out;
        out.reserve(m_slots.size());
        for (std::size_t i = 0; i < m_slots.size(); ++i) {
            if (m_filled[i]) {
                out.push_back(m_slots[i]);
            }
        }
        return out;
    }
};

} // namespace wmtk::threading