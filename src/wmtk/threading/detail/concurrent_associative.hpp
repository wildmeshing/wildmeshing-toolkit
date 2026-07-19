#pragma once

#include <mutex>

#include "../container_range.hpp"

namespace wmtk::threading::detail {

template <typename MapType>
class concurrent_associative
{
protected:
    MapType m_map;
    mutable std::mutex m_mutex;

public:
    using key_type = typename MapType::key_type;
    using mapped_type = typename MapType::mapped_type;
    using value_type = typename MapType::value_type;
    using iterator = typename MapType::iterator;
    using const_iterator = typename MapType::const_iterator;

    concurrent_associative() = default;

    // std::map / std::unordered_map keep references to elements valid across
    // inserts, so returning a reference under a short structural lock is safe to
    // use afterwards without holding the lock (matches TBB semantics; concurrent
    // writes to the *same* element remain the caller's responsibility, as in TBB).
    mapped_type& operator[](const key_type& k)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_map[k];
    }

    iterator find(const key_type& k)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_map.find(k);
    }
    const_iterator find(const key_type& k) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_map.find(k);
    }

    std::size_t count(const key_type& k) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_map.count(k);
    }

    iterator begin() { return m_map.begin(); }
    iterator end() { return m_map.end(); }
    const_iterator begin() const { return m_map.begin(); }
    const_iterator end() const { return m_map.end(); }

    std::size_t size() const { return m_map.size(); }
    bool empty() const { return m_map.empty(); }
    void clear() { m_map.clear(); }

    // Serial "range" over the whole map (see parallel_for(container_range,...)).
    container_range<iterator> range() { return {m_map.begin(), m_map.end()}; }
    container_range<iterator> range(std::size_t) { return {m_map.begin(), m_map.end()}; }
};

} // namespace wmtk::threading::detail