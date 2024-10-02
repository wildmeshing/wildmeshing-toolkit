#pragma once

#include <array>
#include <vector>

namespace wmtk::utils {

constexpr size_t DynamicArraySize = 10; // will be replaced by a template parameter

class DynamicArray
{
public:
    int& operator[](const size_t index);
    const int& operator[](const size_t index) const;

    void emplace_back(const int& val);

    size_t size() const;
    size_t capacity() const;

    void reserve(const size_t new_capacity);

    bool uses_vector() const;

private:
    void switch_to_vector();

private:
    std::array<int, DynamicArraySize> m_array;
    std::vector<int> m_vector;

    bool m_use_vector = false;
    size_t m_end_index = 0;
};

} // namespace wmtk::utils