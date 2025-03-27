#pragma once
#include <wmtk/utils/Rational.hpp>
#include "Accessor.hpp"
#include "MeshAttributeHandle.hpp"

namespace wmtk::attribute {

template <typename T, typename MeshType, typename AttributeType, int Dim>
inline Accessor<T, MeshType, AttributeType, Dim>::Accessor(
    MeshType& m,
    const TypedAttributeHandle<T>& handle)
    : m_handle(handle)
    , m_mesh(m)
    , m_attribute(mesh().m_attribute_manager.get(m_handle).attribute(m_handle.m_base_handle))
{}
template <typename T, typename MeshType, typename AttributeType, int Dim>
Accessor<T, MeshType, AttributeType, Dim>::Accessor(
    const MeshType& m,
    const TypedAttributeHandle<T>& handle)
    : Accessor(const_cast<MeshType&>(m), handle)
{}
template <typename T, typename MeshType, typename AttributeType, int Dim>
template <typename OMType, typename OAT, int D>
Accessor<T, MeshType, AttributeType, Dim>::Accessor(const Accessor<T, OMType, OAT, D>& o)
    : Accessor(static_cast<const MeshType&>(o.mesh()), o.handle())
{
    static_assert(Dim == Eigen::Dynamic || D == Eigen::Dynamic || Dim == D);
}


template <typename T, typename MeshType, typename AttributeType, int Dim>
int64_t Accessor<T, MeshType, AttributeType, Dim>::index(const Tuple& t) const
{
    assert(mesh().is_valid(t));
    return this->mesh().id(t, typed_handle().primitive_type());
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
int64_t Accessor<T, MeshType, AttributeType, Dim>::index(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == primitive_type());
    return this->index(t.tuple());
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
int64_t Accessor<T, MeshType, AttributeType, Dim>::index(const simplex::IdSimplex& t) const
{
    assert(t.primitive_type() == primitive_type());
    return this->mesh().id(t);
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
template <int D, typename ArgType>
auto Accessor<T, MeshType, AttributeType, Dim>::const_vector_attribute(const ArgType& t) const
    -> ConstMapResult<std::max(D, Dim)>
{
    const int64_t idx = this->index(t);
    return m_attribute.template const_vector_attribute<std::max(D, Dim)>(idx);
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
template <int D, typename ArgType>
auto Accessor<T, MeshType, AttributeType, Dim>::vector_attribute(const ArgType& t)
    -> MapResult<std::max(D, Dim)>
{
    const int64_t idx = this->index(t);
    return m_attribute.template vector_attribute<std::max(D, Dim)>(idx);
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
template <typename ArgType>
auto Accessor<T, MeshType, AttributeType, Dim>::scalar_attribute(const ArgType& t) -> Scalar&
{
    const int64_t idx = this->index(t);
    return m_attribute.scalar_attribute(idx);
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
template <typename ArgType>
auto Accessor<T, MeshType, AttributeType, Dim>::const_scalar_attribute(const ArgType& t) const
    -> const Scalar&
{
    const int64_t idx = this->index(t);
    return m_attribute.const_scalar_attribute(idx);
}
// template <typename T, typename MeshType, typename AttributeType, int Dim>
// template <typename ArgType>
// auto Accessor<T, MeshType, AttributeType, Dim>::topological_scalar_attribute(const ArgType& t) ->
// Scalar&
//{
//     const int64_t idx = this->index(t);
//     return m_attribute.topological_scalar_attribute<Dim>(idx);
// }


template <typename T, typename MeshType, typename AttributeType, int Dim>
const T& Accessor<T, MeshType, AttributeType, Dim>::const_topological_scalar_attribute(
    const Tuple& t,
    PrimitiveType pt) const
{
    assert(mesh().top_simplex_type() == m_handle.primitive_type());
    switch (pt) {
    case PrimitiveType::Vertex:
        return m_attribute.const_vector_single_value(t.global_cid(), t.local_vid());
    case PrimitiveType::Edge:
        return m_attribute.const_vector_single_value(t.global_cid(), t.local_eid());
    case PrimitiveType::Triangle:
        return m_attribute.const_vector_single_value(t.global_cid(), t.local_fid());
    case PrimitiveType::Tetrahedron: [[fallthrough]];
    default: {
        const static T x(0);
        return x;
    }
    }
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
inline MeshType& Accessor<T, MeshType, AttributeType, Dim>::mesh()
{
    return m_mesh;
}
template <typename T, typename MeshType, typename AttributeType, int Dim>
inline const MeshType& Accessor<T, MeshType, AttributeType, Dim>::mesh() const
{
    return m_mesh;
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
inline int64_t Accessor<T, MeshType, AttributeType, Dim>::reserved_size() const
{
    return attribute().reserved_size();
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
inline int64_t Accessor<T, MeshType, AttributeType, Dim>::dimension() const
{
    return attribute().dimension();
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
inline auto Accessor<T, MeshType, AttributeType, Dim>::default_value() const -> const Scalar&
{
    return attribute().default_value();
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
inline auto Accessor<T, MeshType, AttributeType, Dim>::attribute() -> AttributeType&
{
    return m_attribute;
}
template <typename T, typename MeshType, typename AttributeType, int Dim>
inline auto Accessor<T, MeshType, AttributeType, Dim>::attribute() const -> const AttributeType&
{
    return m_attribute;
}
template <typename T, typename MeshType, typename AttributeType, int Dim>
inline auto Accessor<T, MeshType, AttributeType, Dim>::typed_handle() const
    -> const TypedAttributeHandle<Scalar>&
{
    return m_handle;
}
template <typename T, typename MeshType, typename AttributeType, int Dim>
inline MeshAttributeHandle Accessor<T, MeshType, AttributeType, Dim>::handle() const
{
    return MeshAttributeHandle(m_mesh, m_handle);
}

template <typename T, typename MeshType, typename AttributeType, int Dim>
inline PrimitiveType Accessor<T, MeshType, AttributeType, Dim>::primitive_type() const
{
    return handle().primitive_type();
}
// template class Accessor<char>;
// template class Accessor<int64_t>;
// template class Accessor<double>;
// template class Accessor<Rational>;
} // namespace wmtk::attribute
