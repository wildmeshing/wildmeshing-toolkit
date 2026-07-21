#pragma once

#include <mutex>
#include <vector>

namespace wmtk::threading {
/**
 * concurrent_vector replaces tbb::concurrent_vector for the "append from a parallel loop, read
 * afterwards" use cases.
 * Writes with `push_back` and `emplace_back` are thread-safe, but reads with `operator[]` are not
 * thread-safe! Reads should only be done after all writes are complete.
 */
template <typename T>
class concurrent_vector
{
    std::vector<T> m_data;
    mutable std::mutex m_mutex;

public:
    using value_type = T;
    using iterator = typename std::vector<T>::iterator;
    using const_iterator = typename std::vector<T>::const_iterator;
    using reference = typename std::vector<T>::reference;
    using const_reference = typename std::vector<T>::const_reference;

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
    void emplace_back(Args&&... args)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.emplace_back(std::forward<Args>(args)...);
    }

    void resize(std::size_t n)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.resize(n);
    }
    void resize(std::size_t n, const T& value)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.resize(n, value);
    }
    void clear()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.clear();
    }
    void reserve(std::size_t n)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.reserve(n);
    }

    /**
     * @brief operator[] is not thread-safe for concurrent writes, but is safe for concurrent reads
     * after all writes are done. Only const reference is provided to prevent accidental writes.
     * @param i index
     * @return const reference to the element at index i
     */
    const_reference operator[](std::size_t i) const { return m_data[i]; }
    // reference operator[](std::size_t i) { return m_data[i]; }

    std::size_t size() const { return m_data.size(); }
    bool empty() const { return m_data.empty(); }

    // iterator begin() { return m_data.begin(); }
    // iterator end() { return m_data.end(); }
    const_iterator begin() const { return m_data.begin(); }
    const_iterator end() const { return m_data.end(); }
};

} // namespace wmtk::threading