#pragma once

#include <array>
#include <vector>

namespace wmtk::utils {


template <typename T, uint64_t ArraySize = 50>
class DynamicArray
{
public:
    class Iterator
    {
    public:
        Iterator(const DynamicArray* container, const uint64_t index = 0);
        Iterator operator++();
        bool operator!=(const Iterator& other) const;
        T operator*();

    private:
        const DynamicArray* m_container;
        uint64_t m_index = 0;
    };


    T& operator[](const uint64_t index);
    const T& operator[](const uint64_t index) const;

    void emplace_back(const T& val);

    uint64_t size() const;
    uint64_t capacity() const;

    /**
     * @brief Return the size of the static array.
     *
     * This function does NOT return the size of the DynamicArray!
     */
    constexpr static uint64_t array_size();

    void reserve(const uint64_t new_capacity);

    bool uses_vector() const;

    Iterator begin() const { return Iterator(this); }
    Iterator end() const { return Iterator(this, m_end_index); }

private:
    void switch_to_vector();

private:
    std::array<T, ArraySize> m_array;
    std::vector<T> m_vector;

    bool m_use_vector = false;
    uint64_t m_end_index = 0;
};

} // namespace wmtk::utils

#include "DynamicArray.hxx"