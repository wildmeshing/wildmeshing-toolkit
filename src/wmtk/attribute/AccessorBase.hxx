#pragma once
#include <wmtk/utils/Rational.hpp>
#include "AccessorBase.hpp"
#include "AttributeManager.hpp"
// #include "MeshAttributeHandle.hpp"
// #include "MeshAttributes.hpp"
//#include "wmtk/Mesh.hpp"

namespace wmtk::attribute {
template <typename T>
AccessorBase<T>::AccessorBase(Mesh& m, const TypedAttributeHandle<T>& handle)
    : m_handle(handle)
    , m_mesh(m)
      , m_attribute(mesh().m_attribute_manager.get(m_handle).attribute(m_handle.m_base_handle))
{}
template <typename T>
AccessorBase<T>::AccessorBase(const Mesh& m, const TypedAttributeHandle<T>& handle): AccessorBase(const_cast<Mesh&>(m), handle) {}


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
int64_t AccessorBase<T>::reserved_size() const
{
    return attribute().reserved_size();
}

template <typename T>
int64_t AccessorBase<T>::dimension() const
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
    return m_attribute;
}
template <typename T>
auto AccessorBase<T>::attribute() const -> const Attribute<T>&
{
    return m_attribute;
}
template <typename T>
const TypedAttributeHandle<T>& AccessorBase<T>::typed_handle() const
{
    return m_handle;
}
template <typename T>
MeshAttributeHandle AccessorBase<T>::handle() const
{
    return MeshAttributeHandle(m_mesh, m_handle);
}

template <typename T>
PrimitiveType AccessorBase<T>::primitive_type() const
{
    return handle().primitive_type();
}

template <typename T>
template <int D>
auto AccessorBase<T>::const_vector_attribute(const int64_t index) const -> ConstMapResult<D>
{
    auto buffer = attribute().template const_vector_attribute<D>(index);
    return buffer;
}

template <typename T>
template <int D>
auto AccessorBase<T>::vector_attribute(const int64_t index) -> MapResult<D>
{
    auto& attr = attribute();
    auto buffer = attr.template vector_attribute<D>(index);

    return buffer;
}

template <typename T>
T AccessorBase<T>::const_scalar_attribute(const int64_t index) const
{
    auto value = attribute().const_scalar_attribute(index);
    return value;
}
template <typename T>
auto AccessorBase<T>::scalar_attribute(const int64_t index) -> T&
{
    auto& value = attribute().scalar_attribute(index);
    return value;
}

template <typename T>
T AccessorBase<T>::const_scalar_attribute(const int64_t index, const int8_t offset) const
{
    auto value = attribute().const_scalar_attribute(index, offset);
    return value;
}
template <typename T>
auto AccessorBase<T>::scalar_attribute(const int64_t index, const int8_t offset) -> T&
{
    auto& value = attribute().scalar_attribute(index, offset);
    return value;
}

template <typename T>
void AccessorBase<T>::set_attribute(std::vector<T> value)
{
    attribute().set(std::move(value));
}
// template class AccessorBase<char>;
// template class AccessorBase<int64_t>;
// template class AccessorBase<double>;
// template class AccessorBase<Rational>;
} // namespace wmtk::attribute
