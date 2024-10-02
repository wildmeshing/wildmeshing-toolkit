#include "DynamicArray.hpp"

#include <assert.h>

#include <wmtk/utils/Logger.hpp>

namespace wmtk::utils {

int& DynamicArray::operator[](const size_t index)
{
    assert(index < m_end_index);
    if (m_use_vector) {
        return m_vector[index];
    } else {
        return m_array[index];
    }
}

const int& DynamicArray::operator[](const size_t index) const
{
    assert(index < m_end_index);
    if (m_use_vector) {
        return m_vector[index];
    } else {
        return m_array[index];
    }
}

void DynamicArray::emplace_back(const int& val)
{
    if (m_use_vector) {
        m_vector.emplace_back(val);
        return;
    }

    if (m_end_index < DynamicArraySize) {
        m_array[m_end_index++] = val;
        return;
    }

    // switch from array to vector
    switch_to_vector();

    m_vector.emplace_back(val);
    ++m_end_index;
}

size_t DynamicArray::size() const
{
    return m_end_index;
}

size_t DynamicArray::capacity() const
{
    if (m_use_vector) {
        return m_vector.capacity();
    }
    return DynamicArraySize;
}

void DynamicArray::reserve(const size_t new_capacity)
{
    if (new_capacity < DynamicArraySize) {
        return;
    }

    if (!m_use_vector) {
        switch_to_vector();
    }

    m_vector.reserve(new_capacity);
}

bool DynamicArray::uses_vector() const
{
    return m_use_vector;
}

void DynamicArray::switch_to_vector()
{
    logger().info("Switching from array to vector.");
    m_use_vector = true;
    m_vector.reserve(DynamicArraySize * 2);
    std::copy(m_array.begin(), m_array.end(), std::back_inserter(m_vector));
}

} // namespace wmtk::utils