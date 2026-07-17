#pragma once

#include <deque>
#include <mutex>

namespace wmtk::threading {
// ---------------------------------------------------------------------------
// concurrent_vector: replaces tbb::concurrent_vector for the "append from a
// parallel loop, read afterwards" use cases. Backed by std::deque so that
// (a) references to existing elements survive concurrent appends, and
// (b) bool is stored unpacked (safe concurrent distinct-index writes).
// ---------------------------------------------------------------------------
template <typename T>
class concurrent_vector
{
    std::deque<T> m_data;
    mutable std::mutex m_mutex;

public:
    using value_type = T;
    using iterator = typename std::deque<T>::iterator;
    using const_iterator = typename std::deque<T>::const_iterator;
    using reference = typename std::deque<T>::reference;
    using const_reference = typename std::deque<T>::const_reference;

    concurrent_vector() = default;
    explicit concurrent_vector(std::size_t n)
        : m_data(n)
    {}
    concurrent_vector(std::size_t n, const T& value)
        : m_data(n, value)
    {}

    concurrent_vector(const concurrent_vector& o) { m_data = o.m_data; }
    concurrent_vector& operator=(const concurrent_vector& o)
    {
        m_data = o.m_data;
        return *this;
    }

    void push_back(const T& v)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.push_back(v);
    }
    void push_back(T&& v)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.push_back(std::move(v));
    }
    template <typename... Args>
    reference emplace_back(Args&&... args)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.emplace_back(std::forward<Args>(args)...);
        return m_data.back();
    }

    // Single-threaded sizing/clearing (used outside parallel regions).
    void resize(std::size_t n) { m_data.resize(n); }
    void resize(std::size_t n, const T& value) { m_data.resize(n, value); }
    void clear() { m_data.clear(); }
    void reserve(std::size_t) {}

    reference operator[](std::size_t i) { return m_data[i]; }
    const_reference operator[](std::size_t i) const { return m_data[i]; }

    std::size_t size() const { return m_data.size(); }
    bool empty() const { return m_data.empty(); }

    iterator begin() { return m_data.begin(); }
    iterator end() { return m_data.end(); }
    const_iterator begin() const { return m_data.begin(); }
    const_iterator end() const { return m_data.end(); }
    const_iterator cbegin() const { return m_data.cbegin(); }
    const_iterator cend() const { return m_data.cend(); }
};

} // namespace wmtk::threading