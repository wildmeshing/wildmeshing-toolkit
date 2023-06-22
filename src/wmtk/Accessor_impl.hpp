#pragma once
#include "Accessor.hpp"
namespace wmtk {

template <typename T>
Accessor<T>::Accessor(Mesh& m, const AttributeHandle& handle)
    : m_attribute(m.template get_mesh_attributes<T>())
    , m_handle(handle)
{}

template <typename T>
ConstMapResult<T> Accessor<T>::vector_attribute(const long index) const
{
    return m_attribute.vector_attribute(m_handle, index);
}
template <typename T>
MapResult<T> Accessor<T>::vector_attribute(const long index)
{
    return.vector_attribute(m_handle, index);
}

template <typename T>
T Accessor<T>::scalar_attribute(const long index) const
{
    return m_attribute.scalar_attribute(m_handle, index);
}

template <typename T>
T& Accessor<T>::scalar_attribute(const long index)
{
    return m_attribute.scalar_attribute(m_handle, index);
}
} // namespace wmtk
