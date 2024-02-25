#pragma once
#include <wmtk/utils/Rational.hpp>
#include "Accessor.hpp"

namespace wmtk::attribute {

template <typename T, typename MeshType>
Accessor<T, MeshType>::Accessor(MeshType& m, const TypedAttributeHandle<T>& h)
    : CachingBaseType(m, h)
{}
template <typename T, typename MeshType>
Accessor<T, MeshType>::Accessor(const MeshType& m, const TypedAttributeHandle<T>& h)
    : CachingBaseType(m, h)
{}
template <typename T, typename MeshType>
template <typename OMType>
Accessor<T, MeshType>::Accessor(const Accessor<T, OMType>& o)
    : Accessor(o.mesh(), o.handle())
{}

template <typename T, typename MeshType>
template <int D>
auto Accessor<T, MeshType>::const_vector_attribute(const Tuple& t) const -> ConstMapResult<D>
{
    const int64_t idx = index(t);
    return CachingBaseType::template const_vector_attribute<D>(idx);
}

template <typename T, typename MeshType>
template <int D>
auto Accessor<T, MeshType>::vector_attribute(const Tuple& t) -> MapResult<D>
{
    const int64_t idx = index(t);
    return CachingBaseType::template vector_attribute<D>(idx);
}

template <typename T, typename MeshType>
auto Accessor<T, MeshType>::scalar_attribute(const Tuple& t) -> T&
{
    const int64_t idx = index(t);
    return CachingBaseType::scalar_attribute(idx);
}

template <typename T, typename MeshType>
T Accessor<T, MeshType>::const_scalar_attribute(const Tuple& t) const
{
    const int64_t idx = index(t);
    return CachingBaseType::const_scalar_attribute(idx);
}
template <typename T, typename MeshType>
int64_t Accessor<T, MeshType>::index(const Tuple& t) const
{
    assert(mesh().is_valid_slow(t));
    return static_cast<const MeshType&>(mesh()).id(t, BaseType::typed_handle().primitive_type());
}

template <typename T, typename MeshType>
auto Accessor<T, MeshType>::topological_scalar_attribute(const Tuple& t) -> T&
{
    const int64_t idx = index(t);
    return CachingBaseType::scalar_attribute(idx);
}

template <typename T, typename MeshType>
T Accessor<T, MeshType>::const_topological_scalar_attribute(const Tuple& t, PrimitiveType pt) const
{
    assert(mesh().top_simplex_type() == BaseType::primitive_type());
    switch (pt) {
    case PrimitiveType::Vertex:
        return CachingBaseType::const_scalar_attribute(t.m_global_cid, t.m_local_vid);
    case PrimitiveType::Edge:
        return CachingBaseType::const_scalar_attribute(t.m_global_cid, t.m_local_eid);
    case PrimitiveType::Triangle:
        return CachingBaseType::const_scalar_attribute(t.m_global_cid, t.m_local_fid);
    case PrimitiveType::Tetrahedron: [[fallthrough]];
    default: return T(0);
    }
}
// template class Accessor<char>;
// template class Accessor<int64_t>;
// template class Accessor<double>;
// template class Accessor<Rational>;
} // namespace wmtk::attribute
