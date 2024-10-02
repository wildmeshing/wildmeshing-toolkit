#pragma once

#include <array>
#include <vector>

namespace wmtk::utils {

constexpr size_t DynamicArraySize = 10; // will be replaced by a template parameter

class DynamicArray
{
public:
    class Iterator
    {
    public:
        Iterator(const DynamicArray* container, const size_t index = 0);
        Iterator operator++();
        bool operator!=(const Iterator& other) const;
        int operator*();

    private:
        const DynamicArray* m_container;
        size_t m_index = 0;
    };


    int& operator[](const size_t index);
    const int& operator[](const size_t index) const;

    void emplace_back(const int& val);

    size_t size() const;
    size_t capacity() const;

    void reserve(const size_t new_capacity);

    bool uses_vector() const;

    Iterator begin() const { return Iterator(this); }
    Iterator end() const { return Iterator(this, m_end_index); }

private:
    void switch_to_vector();

private:
    std::array<int, DynamicArraySize> m_array;
    std::vector<int> m_vector;

    bool m_use_vector = false;
    size_t m_end_index = 0;
};

} // namespace wmtk::utils