#pragma once

#include <bitset>
#include <wmtk/attribute/Accessor.hpp>


namespace wmtk::attribute::internal {


template <size_t N, typename MeshType, typename... AttributeType>
class CompoundAccessor
{
public:
    CompoundAccessor(MeshType& m, const TypedAttributeHandle<AttributeType>&... handle);
    CompoundAccessor(const MeshType& m, const TypedAttributeHandle<AttributeType>&... handle);
    template <typename FirstAcc, int FirstDim, typename... AccTypes, int... Dims>
    CompoundAccessor(
        const Accessor<FirstAcc, MeshType, FirstDim>&,
        const Accessor<AccTypes, MeshType, Dims>&...);

    using AccessorTuple = std::tuple<Accessor<AttributeType, MeshType>...>;


    template <size_t... Ns>
    auto _value(const Tuple& t, std::integer_sequence<size_t, Ns...>)
    {
        return std::make_tuple(std::get<Ns>(m_base_accessors).vector_attribute(t)...);
    }
    auto value(const Tuple& t) { return _value(t, std::make_integer_sequence<size_t, N>{}); }

    template <size_t... Ns>
    auto _const_value(const Tuple& t, std::integer_sequence<size_t, Ns...>) const
    {
        return std::make_tuple(std::get<Ns>(m_base_accessors).const_vector_attribute(t)...);
    }
    auto const_value(const Tuple& t) const
    {
        return _const_value(t, std::make_integer_sequence<size_t, N>{});
    }
    // Eigen::Map<VectorX<T>>
    // template <int D = Eigen::Dynamic>
    // using MapResult = internal::MapResult<Compound, D>;
    //// Eigen::Map<const VectorX<T>>
    // template <int D = Eigen::Dynamic>
    // using ConstMapResult = internal::ConstMapResult<Compound, D>;


    // const & const_scalar_attribute(const Compound& t) const;
    // Compound& scalar_attribute(const Compound& t);

    // template <int D = Eigen::Dynamic>
    // ConstMapResult<D> const_vector_attribute(const Compound& t) const;
    // template <int D = Eigen::Dynamic>
    // MapResult<D> vector_attribute(const Compound& t);

    // Eigen::Index dimension() const { return 1; }

private:
    std::tuple<Accessor<AttributeType, MeshType>...> m_base_accessors;
};


template <typename MeshType, typename... AttrType>
CompoundAccessor(MeshType& m, const TypedAttributeHandle<AttrType>&... handle)
    -> CompoundAccessor<sizeof...(AttrType), MeshType, AttrType...>;
template <typename FirstAcc, typename MeshType, int FirstDim, typename... AccTypes, int... Dims>
CompoundAccessor(
    const Accessor<FirstAcc, MeshType, FirstDim>&,
    const Accessor<AccTypes, MeshType, Dims>&...)
    -> CompoundAccessor<sizeof...(AccTypes) + 1, MeshType, FirstAcc, AccTypes...>;
// template <typename MeshType> CompoundAccessor(const MeshType& m,
// const TypedAttributeHandle<int64_t>& handle)
//     -> CompoundAccessor<MeshType>;

} // namespace wmtk::attribute::internal
#include "CompoundAccessor.hxx"
