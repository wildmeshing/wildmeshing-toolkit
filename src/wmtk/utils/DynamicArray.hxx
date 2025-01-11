#include "DynamicArray.hpp"

#include <assert.h>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::utils {

template <typename T, uint64_t ArraySize>
T& DynamicArray<T, ArraySize>::operator[](const uint64_t index)
{
    assert(index < m_end_index);
    if (m_use_vector) {
        return m_vector[index];
    } else {
        return m_array[index];
    }
}

template <typename T, uint64_t ArraySize>
const T& DynamicArray<T, ArraySize>::operator[](const uint64_t index) const
{
    assert(index < m_end_index);
    if (m_use_vector) {
        return m_vector[index];
    } else {
        return m_array[index];
    }
}

template <typename T, uint64_t ArraySize>
void DynamicArray<T, ArraySize>::emplace_back(const T& val)
{
    if (m_use_vector) {
        m_vector.emplace_back(val);
        m_end_index++;
        return;
    }

    if (m_end_index < ArraySize) {
        m_array[m_end_index++] = val;
        return;
    }

    // switch from array to vector
    switch_to_vector();

    m_vector.emplace_back(val);
    ++m_end_index;
}

template <typename T, uint64_t ArraySize>
uint64_t DynamicArray<T, ArraySize>::size() const
{
    return m_end_index;
}

template <typename T, uint64_t ArraySize>
uint64_t DynamicArray<T, ArraySize>::capacity() const
{
    if (m_use_vector) {
        return m_vector.capacity();
    }
    return ArraySize;
}

template <typename T, uint64_t ArraySize>
inline constexpr uint64_t DynamicArray<T, ArraySize>::array_size()
{
    return ArraySize;
}

template <typename T, uint64_t ArraySize>
void DynamicArray<T, ArraySize>::reserve(const uint64_t new_capacity)
{
    if (new_capacity < ArraySize) {
        return;
    }

    if (!m_use_vector) {
        switch_to_vector();
    }

    m_vector.reserve(new_capacity);
}

template <typename T, uint64_t ArraySize>
bool DynamicArray<T, ArraySize>::uses_vector() const
{
    return m_use_vector;
}

template <typename T, uint64_t ArraySize>
void DynamicArray<T, ArraySize>::switch_to_vector()
{
    logger().debug("Switching from array to vector.");
    m_use_vector = true;
    m_vector.reserve(ArraySize * 2);
    std::copy(m_array.begin(), m_array.end(), std::back_inserter(m_vector));
}

template <typename T, uint64_t ArraySize>
DynamicArray<T, ArraySize>::Iterator::Iterator(const DynamicArray* container, const uint64_t index)
    : m_container(container)
    , m_index(index)
{}

template <typename T, uint64_t ArraySize>
typename DynamicArray<T, ArraySize>::Iterator DynamicArray<T, ArraySize>::Iterator::operator++()
{
    ++m_index;
    return *this;
}

template <typename T, uint64_t ArraySize>
bool DynamicArray<T, ArraySize>::Iterator::operator==(const Iterator& other) const
{
    return m_index == other.m_index;
}

template <typename T, uint64_t ArraySize>
bool DynamicArray<T, ArraySize>::Iterator::operator!=(const Iterator& other) const
{
    return m_index != other.m_index;
}

template <typename T, uint64_t ArraySize>
T DynamicArray<T, ArraySize>::Iterator::operator*()
{
    return (*m_container)[m_index];
}

} // namespace wmtk::utils
