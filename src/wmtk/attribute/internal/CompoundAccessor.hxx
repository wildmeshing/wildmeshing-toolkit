

#pragma once
#include <wmtk/utils/Rational.hpp>
#include "CompoundAccessor.hpp"

namespace wmtk::attribute::internal {


template <size_t N, typename MeshType, typename... AttributeType>
CompoundAccessor<N, MeshType, AttributeType...>::CompoundAccessor(
    const MeshType& m,
    const TypedAttributeHandle<AttributeType>&... handle)
    : m_base_accessors(Accessor<AttributeType, MeshType>(m, handle)...)
{}
template <size_t N, typename MeshType, typename... AttributeType>
CompoundAccessor<N, MeshType, AttributeType...>::CompoundAccessor(
    MeshType& m,
    const TypedAttributeHandle<AttributeType>&... handle)
    : CompoundAccessor(const_cast<const MeshType&>(m), handle...)
{}
template <size_t N, typename MeshType, typename... AttributeType>
template <typename FirstAccType, int FirstDim, typename... AccTypes, int... Dims>
CompoundAccessor<N, MeshType, AttributeType...>::CompoundAccessor(
    const Accessor<FirstAccType, MeshType, FirstDim>& acc,
    const Accessor<AccTypes, MeshType, Dims>&... accs)
    //: m_base_accessors(accs...)
    : CompoundAccessor(acc.mesh(), acc.typed_handle(), accs.typed_handle()...)
{}

template <size_t N, typename MeshType, typename... AttributeType>
template <size_t Index>
PrimitiveType CompoundAccessor<N, MeshType, AttributeType...>::primitive_type() const
{
    return get<Index>().primitive_type();
}
// template <typename MeshType, typename... AttributeType, std::bitset<sizeof...(AttributeType)>
// ScalarAttributeMask> CompoundAccessor<MeshType>::CompoundAccessor(
//{}

// template <typename MeshType, typename... AttributeType, std::bitset<sizeof...(AttributeType)>
// ScalarAttributeMask> template <int D> auto
// CompoundAccessor<MeshType>::const_vector_attribute(const Compound& t) const -> ConstMapResult<D>
//{
//     auto base_map = m_base_accessor.template const_vector_attribute<D>(t);
//
//     const int64_t* int_data = base_map.data();
//     const Compound* data = reinterpret_cast<const Compound*>(int_data);
//     return ConstMapResult<D>(data, dimension());
// }
//
// template <typename MeshType, typename... AttributeType, std::bitset<sizeof...(AttributeType)>
// ScalarAttributeMask> template <int D> auto CompoundAccessor<MeshType>::vector_attribute(const
// Compound& t) -> MapResult<D>
//{
//     auto base_map = m_base_accessor.template vector_attribute<D>(t);
//     int64_t* int_data = base_map.data();
//     Compound* data = reinterpret_cast<Compound*>(int_data);
//     return MapResult<D>(data, dimension());
// }
//
// template <typename MeshType, typename... AttributeType, std::bitset<sizeof...(AttributeType)>
// ScalarAttributeMask> auto CompoundAccessor<MeshType>::scalar_attribute(const Compound& t) ->
// Tuple&
//{
//
//     assert(m_dimension == 1);
//     return *reinterpret_cast<Compound*>(base_map.data());
// }
//
// template <typename MeshType, typename... AttributeType, std::bitset<sizeof...(AttributeType)>
// ScalarAttributeMask> template <typename MeshType> auto
// CompoundAccessor<MeshType>::const_scalar_attribute(const Compound& t) const -> const Tuple&
//{
//     assert(m_dimension == 1);
//     auto base_map = m_base_accessor.template const_vector_attribute(t);
//     return *reinterpret_cast<const Compound*>(base_map.data());
// }


} // namespace wmtk::attribute::internal
