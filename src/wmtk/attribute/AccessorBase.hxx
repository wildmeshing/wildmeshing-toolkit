#pragma once
#include <wmtk/utils/Rational.hpp>
#include "AccessorBase.hpp"
#include "AttributeManager.hpp"
// #include "MeshAttributeHandle.hpp"
// #include "MeshAttributes.hpp"
//#include "wmtk/Mesh.hpp"

namespace wmtk::attribute {
template <typename T, int Dim>
inline AccessorBase<T, Dim>::AccessorBase(Mesh& m, const TypedAttributeHandle<T>& handle)
    : m_handle(handle)
    , m_mesh(m)
    , m_attribute(mesh().m_attribute_manager.get(m_handle).attribute(m_handle.m_base_handle))
{}
template <typename T, int Dim>
AccessorBase<T, Dim>::AccessorBase(const Mesh& m, const TypedAttributeHandle<T>& handle)
    : AccessorBase(const_cast<Mesh&>(m), handle)
{}


template <typename T, int Dim>
inline Mesh& AccessorBase<T, Dim>::mesh()
{
    return m_mesh;
}
template <typename T, int Dim>
inline const Mesh& AccessorBase<T, Dim>::mesh() const
{
    return m_mesh;
}

template <typename T, int Dim>
inline const AttributeManager& AccessorBase<T, Dim>::attribute_manager() const
{
    return mesh().m_attribute_manager;
}

template <typename T, int Dim>
inline AttributeManager& AccessorBase<T, Dim>::attribute_manager()
{
    return mesh().m_attribute_manager;
}


template <typename T, int Dim>
inline AccessorBase<T, Dim>::~AccessorBase() = default;


template <typename T, int Dim>
inline int64_t AccessorBase<T, Dim>::reserved_size() const
{
    return attribute().reserved_size();
}

template <typename T, int Dim>
inline int64_t AccessorBase<T, Dim>::dimension() const
{
    return attribute().dimension();
}

template <typename T, int Dim>
inline const T& AccessorBase<T, Dim>::default_value() const
{
    return attribute().default_value();
}

template <typename T, int Dim>
inline auto AccessorBase<T, Dim>::attributes() -> MeshAttributes<T>&
{
    return attribute_manager().get(m_handle);
}
template <typename T, int Dim>
inline auto AccessorBase<T, Dim>::attributes() const -> const MeshAttributes<T>&
{
    return attribute_manager().get(m_handle);
}
template <typename T, int Dim>
inline auto AccessorBase<T, Dim>::attribute() -> Attribute<T>&
{
    return m_attribute;
}
template <typename T, int Dim>
inline auto AccessorBase<T, Dim>::attribute() const -> const Attribute<T>&
{
    return m_attribute;
}
template <typename T, int Dim>
inline const TypedAttributeHandle<T>& AccessorBase<T, Dim>::typed_handle() const
{
    return m_handle;
}
template <typename T, int Dim>
inline MeshAttributeHandle AccessorBase<T, Dim>::handle() const
{
    return MeshAttributeHandle(m_mesh, m_handle);
}

template <typename T, int Dim>
inline PrimitiveType AccessorBase<T, Dim>::primitive_type() const
{
    return handle().primitive_type();
}

template <typename T, int Dim>
template <int D>
inline auto AccessorBase<T, Dim>::const_vector_attribute(const int64_t index) const
    -> ConstMapResult<D>
{
    auto buffer = attribute().template const_vector_attribute<D>(index);
    return buffer;
}

template <typename T, int Dim>
template <int D>
inline auto AccessorBase<T, Dim>::vector_attribute(const int64_t index) -> MapResult<D>
{
    auto& attr = attribute();
    auto buffer = attr.template vector_attribute<D>(index);

    return buffer;
}

template <typename T, int Dim>
inline T AccessorBase<T, Dim>::const_scalar_attribute(const int64_t index) const
{
    auto value = attribute().const_scalar_attribute(index);
    return value;
}
template <typename T, int Dim>
inline auto AccessorBase<T, Dim>::scalar_attribute(const int64_t index) -> T&
{
    auto& value = attribute().scalar_attribute(index);
    return value;
}

template <typename T, int Dim>
inline T AccessorBase<T, Dim>::const_vector_single_value(const int64_t index, const int8_t offset)
    const
{
    auto value = attribute().template const_vector_single_value<Dim>(index, offset);
    return value;
}
template <typename T, int Dim>
inline auto AccessorBase<T, Dim>::vector_single_value(const int64_t index, const int8_t offset) -> T&
{
    auto& value = attribute().template vector_single_value<Dim>(index, offset);
    return value;
}

template <typename T, int Dim>
inline void AccessorBase<T, Dim>::set_attribute(std::vector<T> value)
{
    attribute().set(std::move(value));
}
// template class AccessorBase<char>;
// template class AccessorBase<int64_t>;
// template class AccessorBase<double>;
// template class AccessorBase<Rational>;
} // namespace wmtk::attribute
