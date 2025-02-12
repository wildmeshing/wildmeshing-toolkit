
#pragma once
#include <wmtk/utils/Rational.hpp>
#include "TupleAccessor.hpp"

namespace wmtk::attribute {


template <typename MeshType, int Dim>
TupleAccessor<MeshType, Dim>::TupleAccessor(
    MeshType& m,
    const TypedAttributeHandle<int64_t>& handle)
    : m_base_accessor(m, handle)
    , m_dimension(m_base_accessor.dimension() / (sizeof(Tuple) / sizeof(int64_t)))
{}
template <typename MeshType, int Dim>
TupleAccessor<MeshType, Dim>::TupleAccessor(
    const MeshType& m,
    const TypedAttributeHandle<int64_t>& handle)
    : TupleAccessor(const_cast<MeshType&>(m), handle)
{}
template <typename MeshType, int Dim>
template <int Dim2>
TupleAccessor<MeshType, Dim>::TupleAccessor(
    const Accessor<int64_t, MeshType, CachingAttribute<int64_t>, Dim2>& accessor)
    : TupleAccessor(accessor.mesh(), accessor.typed_handle())
{
    static_assert(Dim == Eigen::Dynamic || Dim2 == Eigen::Dynamic || Dim == Dim2);
}

template <typename MeshType, int Dim>
template <int D>
auto TupleAccessor<MeshType, Dim>::const_vector_attribute(const Tuple& t) const -> ConstMapResult<D>
{
    auto base_map = m_base_accessor.template const_vector_attribute<D>(t);

    const int64_t* int_data = base_map.data();
    const Tuple* data = reinterpret_cast<const Tuple*>(int_data);
    return ConstMapResult<D>(data, dimension());
}

template <typename MeshType, int Dim>
template <int D>
auto TupleAccessor<MeshType, Dim>::vector_attribute(const Tuple& t) -> MapResult<D>
{
    auto base_map = m_base_accessor.template vector_attribute<D>(t);
    int64_t* int_data = base_map.data();
    Tuple* data = reinterpret_cast<Tuple*>(int_data);
    return MapResult<D>(data, dimension());
}

template <typename MeshType, int Dim>
auto TupleAccessor<MeshType, Dim>::scalar_attribute(const Tuple& t) -> Tuple&
{
    auto base_map = m_base_accessor.template vector_attribute<2>(t);

    assert(m_dimension == 1);
    return *reinterpret_cast<Tuple*>(base_map.data());
}

template <typename MeshType, int Dim>
auto TupleAccessor<MeshType, Dim>::const_scalar_attribute(const Tuple& t) const -> const Tuple&
{
    assert(m_dimension == 1);
    auto base_map = m_base_accessor.template const_vector_attribute<2>(t);
    return *reinterpret_cast<const Tuple*>(base_map.data());
}


} // namespace wmtk::attribute
