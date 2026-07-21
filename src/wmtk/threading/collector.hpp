#pragma once

#include <mutex>
#include <vector>

namespace wmtk::threading {
/**
 * collector replaces tbb::concurrent_vector for the "append from a parallel loop, read
 * afterwards" use cases.
 * Writes with `push_back` and `emplace_back` are thread-safe, but reads with `operator[]` are not
 * thread-safe! Reads should only be done after all writes are complete.
 */
template <typename T>
class collector
{
    std::vector<T> m_data;
    mutable std::mutex m_mutex;

public:
    using value_type = T;
    using iterator = typename std::vector<T>::iterator;
    using const_iterator = typename std::vector<T>::const_iterator;
    using reference = typename std::vector<T>::reference;
    using const_reference = typename std::vector<T>::const_reference;

    collector() = default;
    explicit collector(std::size_t n)
        : m_data(n)
    {}
    collector(std::size_t n, const T& value)
        : m_data(n, value)
    {}

    collector(const collector& o) { m_data = o.m_data; }
    collector& operator=(const collector& o)
    {
        m_data = o.m_data;
        return *this;
    }

    /**
     * @brief Push an element to the end of the collector.
     * This function is thread-safe.
     * @param v The element to push.
     */
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

    /**
     * @brief Emplace an element at the end of the collector.
     * This function is thread-safe.
     * @param args The arguments to forward to the constructor of T.
     */
    template <typename... Args>
    void emplace_back(Args&&... args)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.emplace_back(std::forward<Args>(args)...);
    }

    /**
     * @brief Resize the collector to contain n elements.
     * This function is thread-safe.
     * @param n The new size of the collector.
     */
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
    /**
     * @brief Clear the collector.
     * This function is thread-safe.
     */
    void clear()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.clear();
    }
    /**
     * @brief Reserve space in the collector.
     * This function is thread-safe.
     * @param n The number of elements to reserve space for.
     */
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

    /**
     * @brief Get the number of elements in the collector.
     * This function is NOT thread-safe.
     * @return The number of elements in the collector.
     */
    std::size_t size() const { return m_data.size(); }
    /**
     * @brief Check if the collector is empty.
     * This function is NOT thread-safe.
     * @return True if the collector is empty, false otherwise.
     */
    bool empty() const { return m_data.empty(); }

    // iterator begin() { return m_data.begin(); }
    // iterator end() { return m_data.end(); }
    const_iterator begin() const { return m_data.begin(); }
    const_iterator end() const { return m_data.end(); }
};

} // namespace wmtk::threading