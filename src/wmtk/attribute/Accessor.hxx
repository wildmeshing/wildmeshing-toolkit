#pragma once
#include <wmtk/utils/Rational.hpp>
#include "Accessor.hpp"

namespace wmtk::attribute {

template <typename T>
    template <int D>
auto Accessor<T>::const_vector_attribute(const Tuple& t) const -> ConstMapResult<D>
{
    const int64_t idx = index(t);
    return CachingBaseType::template const_vector_attribute<D>(idx);
}

template <typename T>
    template <int D>
auto Accessor<T>::vector_attribute(const Tuple& t) -> MapResult<D>
{
    const int64_t idx = index(t);
    return CachingBaseType::template vector_attribute<D>(idx);
}

template <typename T>
auto Accessor<T>::scalar_attribute(const Tuple& t) -> T&
{
    const int64_t idx = index(t);
    return CachingBaseType::scalar_attribute(idx);
}

template <typename T>
T Accessor<T>::const_scalar_attribute(const Tuple& t) const
{
    const int64_t idx = index(t);
    return CachingBaseType::const_scalar_attribute(idx);
}
template <typename T>
int64_t Accessor<T>::index(const Tuple& t) const
{
    assert(mesh().is_valid_slow(t));
    return mesh().id(t, BaseType::typed_handle().primitive_type());
}

template <typename T>
auto Accessor<T>::topological_scalar_attribute(const Tuple& t) -> T&
{
    const int64_t idx = index(t);
    return CachingBaseType::scalar_attribute(idx);
}

template <typename T>
T Accessor<T>::const_topological_scalar_attribute(const Tuple& t, PrimitiveType pt) const
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
