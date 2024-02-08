#pragma once
#include <wmtk/utils/Rational.hpp>
#include "TupleAccessor.hpp"

namespace wmtk::attribute {

template <typename T>
inline auto TupleAccessor<T>::const_vector_attribute(const Tuple& t) const -> ConstMapResult
{
    const int64_t idx = index(t);
    return CachingBaseType::const_vector_attribute(idx);
}

template <typename T>
inline auto TupleAccessor<T>::vector_attribute(const Tuple& t) -> MapResult
{
    const int64_t idx = index(t);
    return CachingBaseType::vector_attribute(idx);
}

template <typename T>
inline auto TupleAccessor<T>::scalar_attribute(const Tuple& t) -> T&
{
    const int64_t idx = index(t);
    return CachingBaseType::scalar_attribute(idx);
}

template <typename T>
inline T TupleAccessor<T>::const_scalar_attribute(const Tuple& t) const
{
    const int64_t idx = index(t);
    return CachingBaseType::const_scalar_attribute(idx);
}
template <typename T>
inline int64_t TupleAccessor<T>::index(const Tuple& t) const
{
    assert(mesh().is_valid_slow(t));
    return mesh().id(t, BaseType::typed_handle().primitive_type());
}

template <typename T>
inline auto TupleAccessor<T>::topological_scalar_attribute(const Tuple& t) -> T&
{
    const int64_t idx = index(t);
    return CachingBaseType::scalar_attribute(idx);
}

template <typename T>
inline T TupleAccessor<T>::const_topological_scalar_attribute(const Tuple& t, PrimitiveType pt)
    const
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
// template class TupleAccessor<char>;
// template class TupleAccessor<int64_t>;
// template class TupleAccessor<double>;
// template class TupleAccessor<Rational>;
} // namespace wmtk::attribute
