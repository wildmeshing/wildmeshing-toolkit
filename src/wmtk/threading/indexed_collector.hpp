#pragma once

#include <vector>

namespace wmtk::threading {

/**
 * A collector that is not thread-safe for concurrent writes to the same index, but is thread-safe
 * for concurrent writes to distinct indices. This is useful when each thread knows a unique slot
 * for its result, allowing for lock-free writes.
 */
template <typename T>
class indexed_collector
{
    std::vector<T> m_slots;
    std::vector<char> m_filled;

public:
    /**
     * @brief Construct an indexed_collector with n slots.
     * @param n The number of slots to allocate.
     */
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

    /**
     * @brief Get a compacted vector of all filled slots.
     * This function is not thread-safe and should only be called after all writes are complete.
     *
     * @return A vector containing all values that have been set, in the order of their indices.
     */
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