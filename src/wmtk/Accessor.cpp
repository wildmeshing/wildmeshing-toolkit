#include "Accessor.hpp"

#include "Mesh.hpp"
#include "MeshAttributes.hpp"

namespace wmtk {

template <typename T, bool IsConst>
Accessor<T, IsConst>::Accessor(MeshType& mesh, const MeshAttributeHandle<T>& handle)
    : m_mesh(mesh)
    , m_handle(handle)
{}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::attributes() -> MeshAttributesType&
{
    return m_mesh.get_mesh_attributes(m_handle);
}
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::attributes() const -> const MeshAttributesType&
{
    return m_mesh.get_mesh_attributes(m_handle);
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const long index) const -> ConstMapResult
{
    return attributes().vector_attribute(m_handle.m_base_handle, index);
}
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const long index) -> MapResultT
{
    return attributes().vector_attribute(m_handle.m_base_handle, index);
}

template <typename T, bool IsConst>
T Accessor<T, IsConst>::scalar_attribute(const long index) const
{
    return attributes().scalar_attribute(m_handle.m_base_handle, index);
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::scalar_attribute(const long index) -> TT
{
    return attributes().scalar_attribute(m_handle.m_base_handle, index);
}
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const Tuple& t) const -> ConstMapResult
{
    long index = m_mesh.id(t, m_handle.m_primitive_type);
    return vector_attribute(index);
}
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const Tuple& t) -> MapResultT
{
    long index = m_mesh.id(t, m_handle.m_primitive_type);
    return vector_attribute(index);
}

template <typename T, bool IsConst>
T Accessor<T, IsConst>::scalar_attribute(const Tuple& t) const
{
    long index = m_mesh.id(t, m_handle.m_primitive_type);
    return scalar_attribute(index);
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::scalar_attribute(const Tuple& t) -> TT
{
    long index = m_mesh.id(t, m_handle.m_primitive_type);
    return scalar_attribute(index);
}
// template <typename T, bool IsConst>
// void MeshAttributes<T>::rollback()
//{
//     attributes()s_copy.clear();
// }
//
// template <typename T, bool IsConst>
// void MeshAttributes<T>::begin_protect()
//{
//     attributes()s_copy =
//     attributes()s;
// }
//
// template <typename T, bool IsConst>
// void MeshAttributes<T>::end_protect()
//{
//     if (!attributes()s_copy.empty())
//     attributes()s =
//     std::move(attributes()s_copy);
//
//     attributes()s_copy.clear();
// }
//
// template <typename T, bool IsConst>
// bool MeshAttributes<T>::is_in_protect() const
//{
//     return !attributes()s_copy.empty();
// }
template class Accessor<char, true>;
template class Accessor<long, true>;
template class Accessor<double, true>;
template class Accessor<char, false>;
template class Accessor<long, false>;
template class Accessor<double, false>;
} // namespace wmtk
