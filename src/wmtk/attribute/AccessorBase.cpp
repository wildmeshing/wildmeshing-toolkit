#include "AccessorBase.hpp"
#include <iostream>
#include "wmtk/Mesh.hpp"
#include "wmtk/MeshAttributes.hpp"

namespace wmtk {
template <typename T>
AccessorBase<T>::AccessorBase(Mesh& m, const MeshAttributeHandle<T>& handle)
    : m_mesh(m)
    , m_handle(handle)
{}

template <typename T>
AccessorBase<T>::~AccessorBase() = default;

template <typename T>
long AccessorBase<T>::size() const
{
    return attribute().size();
}

template <typename T>
long AccessorBase<T>::stride() const
{
    return attribute().stride();
}

template <typename T>
auto AccessorBase<T>::attributes() -> MeshAttributes<T>&
{
    return m_mesh.get_mesh_attributes(m_handle);
}
template <typename T>
auto AccessorBase<T>::attributes() const -> const MeshAttributes<T>&
{
    return m_mesh.get_mesh_attributes(m_handle);
}
template <typename T>
auto AccessorBase<T>::attribute() -> AttributeType&
{
    return attributes().attribute(m_handle.m_base_handle);
}
template <typename T>
auto AccessorBase<T>::attribute() const -> const Attribute<T>&
{
    return attributes().attribute(m_handle.m_base_handle);
}

template <typename T>
auto AccessorBase<T>::vector_attribute(const long index) const -> ConstMapResult
{
    return const_vector_attribute(index);
}

template <typename T>
auto AccessorBase<T>::const_vector_attribute(const long index) const -> ConstMapResult
{
    auto buffer = attribute().const_vector_attribute(index);
    return buffer;
}

template <typename T>
auto AccessorBase<T>::vector_attribute(const long index) -> MapResultT
{
    if constexpr (IsConst) {
        auto buffer = attribute().const_vector_attribute(index);

        return buffer;
    } else {
        auto& attr = attribute();
        auto buffer = attr.vector_attribute(index);

        return buffer;
    }
}

template <typename T>
T AccessorBase<T>::const_scalar_attribute(const long index) const
{
    auto value = attribute().const_scalar_attribute(index);
    return value;
}
template <typename T>
auto AccessorBase<T>::scalar_attribute(const long index) -> TT
{
    if constexpr (IsConst) {
        return attribute().const_scalar_attribute(index);
    } else {
        return attribute().scalar_attribute(index);
    }
}


template <typename T>
long AccessorBase<T>::index(const Tuple& t) const
{
    return m_mesh.id(t, m_handle.m_primitive_type);
}
template <typename T>
void AccessorBase<T>::set_attribute(std::vector<T> value)
{
    if constexpr (IsConst) {
        throw std::runtime_error("You cant modify a constant accessor");
    } else {
        attribute().set(std::move(value));
    }
}
template class AccessorBase<char>;
template class AccessorBase<long>;
template class AccessorBase<double>;
} // namespace wmtk
