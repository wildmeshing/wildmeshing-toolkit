#pragma once
#include <wmtk/utils/Rational.hpp>
#include "Accessor.hpp"

namespace wmtk::attribute {

template <typename T, typename MeshType, int Dim>
Accessor<T, MeshType, Dim>::Accessor(MeshType& m, const TypedAttributeHandle<T>& h)
    : CachingBaseType(m, h)
{}
template <typename T, typename MeshType, int Dim>
Accessor<T, MeshType, Dim>::Accessor(const MeshType& m, const TypedAttributeHandle<T>& h)
    : CachingBaseType(m, h)
{}
template <typename T, typename MeshType, int Dim>
template <typename OMType, int D>
Accessor<T, MeshType, Dim>::Accessor(const Accessor<T, OMType, D>& o)
    : Accessor(static_cast<const MeshType&>(o.mesh()), o.handle())
{
    static_assert(Dim == Eigen::Dynamic || D == Eigen::Dynamic || Dim == D);
}

/*
template <typename T, typename MeshType, int Dim>
template <int D>
auto Accessor<T, MeshType, Dim>::const_vector_attribute(const Tuple& t) const -> ConstMapResult<D>
{
    const int64_t idx = this->index(t);
    return CachingBaseType::template const_vector_attribute<D>(idx);
}

template <typename T, typename MeshType, int Dim>
template <int D>
auto Accessor<T, MeshType, Dim>::vector_attribute(const Tuple& t) -> MapResult<D>
{
    const int64_t idx = this->index(t);
    return CachingBaseType::template vector_attribute<D>(idx);
}

template <typename T, typename MeshType, int Dim>
auto Accessor<T, MeshType, Dim>::scalar_attribute(const Tuple& t) -> T&
{
    const int64_t idx = this->index(t);
    return CachingBaseType::scalar_attribute(idx);
}

template <typename T, typename MeshType, int Dim>
T Accessor<T, MeshType, Dim>::const_scalar_attribute(const Tuple& t) const
{
    const int64_t idx = this->index(t);
    return CachingBaseType::const_scalar_attribute(idx);
}

template <typename T, typename MeshType, int Dim>
auto Accessor<T, MeshType, Dim>::topological_scalar_attribute(const Tuple& t) -> T&
{
    const int64_t idx = this->index(t);
    return CachingBaseType::scalar_attribute(idx);
}
*/

template <typename T, typename MeshType, int Dim>
int64_t Accessor<T, MeshType, Dim>::index(const Tuple& t) const
{
    assert(mesh().is_valid(t));
    return this->mesh().id(t, BaseType::typed_handle().primitive_type());
}

template <typename T, typename MeshType, int Dim>
int64_t Accessor<T, MeshType, Dim>::index(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == primitive_type());
    return this->index(t.tuple());

}

template <typename T, typename MeshType, int Dim>
int64_t Accessor<T, MeshType, Dim>::index(const simplex::IdSimplex& t) const
{
    assert(t.primitive_type() == primitive_type());
    return this->mesh().id(t);

}

template <typename T, typename MeshType, int Dim>
template <int D, typename ArgType>
auto Accessor<T, MeshType, Dim>::const_vector_attribute(const ArgType& t) const
    -> ConstMapResult<D>
{
    const int64_t idx = this->index(t);
    return CachingBaseType::template const_vector_attribute<D>(idx);
}

template <typename T, typename MeshType, int Dim>
template <int D, typename ArgType>
auto Accessor<T, MeshType, Dim>::vector_attribute(const ArgType& t) -> MapResult<D>
{
    const int64_t idx = this->index(t);
    return CachingBaseType::template vector_attribute<D>(idx);
}

template <typename T, typename MeshType, int Dim>
template <typename ArgType>
auto Accessor<T, MeshType, Dim>::scalar_attribute(const ArgType& t) -> T&
{
    const int64_t idx = this->index(t);
    return CachingBaseType::scalar_attribute(idx);
}

template <typename T, typename MeshType, int Dim>
template <typename ArgType>
T Accessor<T, MeshType, Dim>::const_scalar_attribute(const ArgType& t) const
{
    const int64_t idx = this->index(t);
    return CachingBaseType::const_scalar_attribute(idx);
}
template <typename T, typename MeshType, int Dim>
template <typename ArgType>
auto Accessor<T, MeshType, Dim>::topological_scalar_attribute(const ArgType& t) -> T&
{
    const int64_t idx = this->index(t);
    return CachingBaseType::scalar_attribute(idx);
}




template <typename T, typename MeshType, int Dim>
T Accessor<T, MeshType, Dim>::const_topological_scalar_attribute(const Tuple& t, PrimitiveType pt)
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
// template class Accessor<char>;
// template class Accessor<int64_t>;
// template class Accessor<double>;
// template class Accessor<Rational>;
} // namespace wmtk::attribute
