#include "AccessorBase.hpp"
#include <iostream>
#include <wmtk/utils/Rational.hpp>
#include "AttributeManager.hpp"
#include "MeshAttributes.hpp"
#include "wmtk/Mesh.hpp"

namespace wmtk::attribute {
template <typename T>
AccessorBase<T>::AccessorBase(Mesh& m, const MeshAttributeHandle<T>& handle)
    : m_mesh(m)
    , m_handle(handle)
{}

template <typename T>
Mesh& AccessorBase<T>::mesh()
{
    return m_mesh;
}
template <typename T>
const Mesh& AccessorBase<T>::mesh() const
{
    return m_mesh;
}

template <typename T>
const AttributeManager& AccessorBase<T>::attribute_manager() const
{
    return mesh().m_attribute_manager;
}

template <typename T>
AttributeManager& AccessorBase<T>::attribute_manager()
{
    return mesh().m_attribute_manager;
}


template <typename T>
AccessorBase<T>::~AccessorBase() = default;


template <typename T>
long AccessorBase<T>::reserved_size() const
{
    return attribute().reserved_size();
}

template <typename T>
long AccessorBase<T>::dimension() const
{
    return attribute().dimension();
}

template <typename T>
auto AccessorBase<T>::attributes() -> MeshAttributes<T>&
{
    return attribute_manager().get(m_handle);
}
template <typename T>
auto AccessorBase<T>::attributes() const -> const MeshAttributes<T>&
{
    return attribute_manager().get(m_handle);
}
template <typename T>
auto AccessorBase<T>::attribute() -> Attribute<T>&
{
    return attributes().attribute(m_handle.m_base_handle);
}
template <typename T>
auto AccessorBase<T>::attribute() const -> const Attribute<T>&
{
    return attributes().attribute(m_handle.m_base_handle);
}
template <typename T>
const MeshAttributeHandle<T>& AccessorBase<T>::handle() const
{
    return m_handle;
}

template <typename T>
PrimitiveType AccessorBase<T>::primitive_type() const
{
    return handle().m_primitive_type;
}

template <typename T>
auto AccessorBase<T>::const_vector_attribute(const long index) const -> ConstMapResult
{
    auto buffer = attribute().const_vector_attribute(index);
    return buffer;
}

template <typename T>
auto AccessorBase<T>::vector_attribute(const long index) -> MapResult
{
    auto& attr = attribute();
    auto buffer = attr.vector_attribute(index);

    return buffer;
}

template <typename T>
T AccessorBase<T>::const_scalar_attribute(const long index) const
{
    auto value = attribute().const_scalar_attribute(index);
    return value;
}
template <typename T>
auto AccessorBase<T>::scalar_attribute(const long index) -> T&
{
    auto& value = attribute().scalar_attribute(index);
    return value;
}


template <typename T>
void AccessorBase<T>::set_attribute(std::vector<T> value)
{
    attribute().set(std::move(value));
}
template class AccessorBase<char>;
template class AccessorBase<long>;
template class AccessorBase<double>;
template class AccessorBase<Rational>;
} // namespace wmtk::attribute
