#include "AccessorBase.hpp"
#include <iostream>
#include "Mesh.hpp"
#include "MeshAttributes.hpp"

namespace wmtk {
template <typename T, bool IsConst>
AccessorBase<T, IsConst>::AccessorBase(MeshType& m, const MeshAttributeHandle<T>& handle)
    : m_mesh(m)
    , m_handle(handle)
{}

template <typename T, bool IsConst>
AccessorBase<T, IsConst>::~AccessorBase() = default;

template <typename T, bool IsConst>
long AccessorBase<T, IsConst>::size() const {
    return attribute().size();
}

template <typename T, bool IsConst>
long AccessorBase<T, IsConst>::stride() const {
    return attribute().stride();
}

template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::attributes() -> MeshAttributesType&
{
    return m_mesh.get_mesh_attributes(m_handle);
}
template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::attributes() const -> const MeshAttributesType&
{
    return m_mesh.get_mesh_attributes(m_handle);
}
template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::attribute() -> AttributeType&
{
    return attributes().attribute(m_handle.m_base_handle);
}
template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::attribute() const -> const Attribute<T>&
{
    return attributes().attribute(m_handle.m_base_handle);
}

template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::vector_attribute(const long index) const -> ConstMapResult
{
    return const_vector_attribute(index);
}

template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::const_vector_attribute(const long index) const -> ConstMapResult
{
    auto buffer = attribute().const_vector_attribute(index);
    return buffer;
}

template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::vector_attribute(const long index) -> MapResultT
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

template <typename T, bool IsConst>
T AccessorBase<T, IsConst>::const_scalar_attribute(const long index) const
{
    auto value = attribute().const_scalar_attribute(index);
    return value;
}
template <typename T, bool IsConst>
T AccessorBase<T, IsConst>::scalar_attribute(const long index) const
{
    return const_scalar_attribute(index);
}

template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::scalar_attribute(const long index) -> TT
{
    if constexpr (IsConst) {
        auto value = attribute().const_scalar_attribute(index);
        return value;
    } else {
        auto& value = attribute().scalar_attribute(index);
        return value;
    }
}

template <typename T, bool IsConst>
long AccessorBase<T, IsConst>::index(const Tuple& t) const
{
    return m_mesh.id(t, m_handle.m_primitive_type);
}
template <typename T, bool IsConst>
void AccessorBase<T, IsConst>::set_attribute(std::vector<T> value)
{
    if constexpr (IsConst) {
        throw std::runtime_error("You cant modify a constant accessor");
    } else {
        attribute().set(std::move(value));
    }
}
template class AccessorBase<char, true>;
template class AccessorBase<long, true>;
template class AccessorBase<double, true>;
template class AccessorBase<char, false>;
template class AccessorBase<long, false>;
template class AccessorBase<double, false>;
} // namespace wmtk
