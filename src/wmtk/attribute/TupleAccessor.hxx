
#pragma once
#include <wmtk/utils/Rational.hpp>
#include "TupleAccessor.hpp"

namespace wmtk::attribute {


template <typename MeshType>
TupleAccessor<MeshType>::TupleAccessor(MeshType& m, const TypedAttributeHandle<int64_t>& handle)
    : m_base_accessor(m, handle)
    , m_dimension(m_base_accessor.dimension() / (sizeof(Tuple) / sizeof(int64_t)))
{}
template <typename MeshType>
TupleAccessor<MeshType>::TupleAccessor(
    const MeshType& m,
    const TypedAttributeHandle<int64_t>& handle)
    : TupleAccessor(const_cast<MeshType&>(m), handle)
{}
template <typename MeshType>
TupleAccessor<MeshType>::TupleAccessor(const Accessor<int64_t, MeshType>& accessor)
    : TupleAccessor(accessor.mesh(), accessor.typed_handle())
{}

template <typename MeshType>
template <int D>
auto TupleAccessor<MeshType>::const_vector_attribute(const Tuple& t) const -> ConstMapResult<D>
{
    auto base_map = m_base_accessor.template const_vector_attribute<D>(t);

    const int64_t* int_data = base_map.data();
    const Tuple* data = reinterpret_cast<const Tuple*>(int_data);
    return ConstMapResult<D>(data, dimension());
}

template <typename MeshType>
template <int D>
auto TupleAccessor<MeshType>::vector_attribute(const Tuple& t) -> MapResult<D>
{
    auto base_map = m_base_accessor.template vector_attribute<D>(t);
    int64_t* int_data = base_map.data();
    Tuple* data = reinterpret_cast<Tuple*>(int_data);
    return MapResult<D>(data, dimension());
}

template <typename MeshType>
auto TupleAccessor<MeshType>::scalar_attribute(const Tuple& t) -> Tuple&
{
    auto base_map = m_base_accessor.template vector_attribute(t);

    assert(m_dimension == 1);
    return *reinterpret_cast<Tuple*>(base_map.data());
}

template <typename MeshType>
auto TupleAccessor<MeshType>::const_scalar_attribute(const Tuple& t) const -> const Tuple&
{
    assert(m_dimension == 1);
    auto base_map = m_base_accessor.template const_vector_attribute(t);
    return *reinterpret_cast<const Tuple*>(base_map.data());
}


} // namespace wmtk::attribute
