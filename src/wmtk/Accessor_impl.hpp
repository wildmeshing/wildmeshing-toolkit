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
// template <typename T>
// void MeshAttributes<T>::rollback()
//{
//     m_attributes_copy.clear();
// }
//
// template <typename T>
// void MeshAttributes<T>::begin_protect()
//{
//     m_attributes_copy = m_attributes;
// }
//
// template <typename T>
// void MeshAttributes<T>::end_protect()
//{
//     if (!m_attributes_copy.empty()) m_attributes = std::move(m_attributes_copy);
//
//     m_attributes_copy.clear();
// }
//
// template <typename T>
// bool MeshAttributes<T>::is_in_protect() const
//{
//     return !m_attributes_copy.empty();
// }
} // namespace wmtk
