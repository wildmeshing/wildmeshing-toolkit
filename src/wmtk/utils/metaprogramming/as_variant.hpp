#pragma once

#include "DerivedReferenceWrapperVariantTraits.hpp"
namespace wmtk::utils::metaprogramming {
namespace detail {

template <typename BaseVariantTraitsType, typename TupleType, size_t Index>
struct as_variant_impl
{
    using RetType = typename BaseVariantTraitsType::ReferenceVariant;
    using BaseType = typename BaseVariantTraitsType::BaseType;
    static RetType exec(BaseType& value, size_t index);
};
template <typename BaseVariantTraitsType, typename... DerivedTypes, size_t Index>
struct as_variant_impl<BaseVariantTraitsType, std::tuple<DerivedTypes...>, Index>
{
    using RetType = typename BaseVariantTraitsType::ReferenceVariant;
    using BaseType = typename BaseVariantTraitsType::BaseType;
    using TupleType = typename BaseVariantTraitsType::DerivedTypesTuple;
    using RefTupleType = typename BaseVariantTraitsType::ReferenceTuple;

    using MyRefType = std::tuple_element_t<Index, RefTupleType>;
    using MyDerivedType = std::tuple_element_t<Index, TupleType>;

    using FirstRefType = std::tuple_element_t<0, RefTupleType>;
    using FirstDerivedType = std::tuple_element_t<0, TupleType>;
    static RetType exec(BaseType& value, size_t index)
    {
        // handle case that the type isn't found / settle base case
        if constexpr (Index < sizeof...(DerivedTypes)) {
            if (index == Index) {
                return RetType(
                    std::in_place_type_t<MyRefType>{},
                    static_cast<MyDerivedType&>(value));
            } else {
                if constexpr (Index + 1 < sizeof...(DerivedTypes)) {
                    return as_variant_impl<BaseVariantTraitsType, TupleType, Index + 1>::exec(
                        value,
                        index);
                }
            }
        }
        throw "Invalid Input";
        // This should never happen, just making a dummy to suppress warnings
        return RetType(
            std::in_place_type_t<FirstRefType>{},
            reinterpret_cast<FirstDerivedType&>(value));
    }
};
} // namespace detail

template <typename BaseVariantTraitsType>
typename BaseVariantTraitsType::ReferenceVariant as_variant(
    typename BaseVariantTraitsType::BaseType& value)
{
    using impl = detail::as_variant_impl<
        BaseVariantTraitsType,
        typename BaseVariantTraitsType::DerivedTypesTuple,
        0>;
    return impl::exec(value, BaseVariantTraitsType::get_index(value));
}
} // namespace wmtk::utils::metaprogramming
