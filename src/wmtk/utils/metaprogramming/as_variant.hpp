#pragma once

#include "DerivedReferenceWrapperVariantTraits.hpp"
namespace wmtk::utils::metaprogramming {
namespace detail {

template <typename BaseVariantTraitsType, bool IsConst, typename TupleType, size_t Index>
struct as_variant_impl
{
    using RetType = typename BaseVariantTraitsType::ReferenceVariant;
    using BaseType = typename BaseVariantTraitsType::BaseType;
    static RetType exec(BaseType& value, size_t index);
};
template <typename BaseVariantTraitsType, bool IsConst, typename... DerivedTypes, size_t Index>
struct as_variant_impl<BaseVariantTraitsType, IsConst, std::tuple<DerivedTypes...>, Index>
{
    using BaseType_ = typename BaseVariantTraitsType::BaseType;
    using BaseType = std::conditional_t<IsConst, const BaseType_, BaseType_>;

    using TupleType = typename BaseVariantTraitsType::DerivedTypesTuple;

    using RetType = typename BaseVariantTraitsType::template ReferenceVariant_const<IsConst>;

    using RefTupleType = typename BaseVariantTraitsType::template ReferenceTuple_const<IsConst>;

    using MyRefType = std::tuple_element_t<Index, RefTupleType>;
    using MyDerivedType = std::tuple_element_t<Index, TupleType>;

    using FirstRefType = std::tuple_element_t<0, RefTupleType>;
    using FirstDerivedType = std::tuple_element_t<0, TupleType>;
    static auto exec(BaseType& value, size_t index) -> RetType
    {
        // handle case that the type isn't found / settle base case
        if constexpr (Index < sizeof...(DerivedTypes)) {
            if (index == Index) {
                return RetType(
                    std::in_place_type_t<MyRefType>{},
                    static_cast<std::conditional_t<IsConst, const MyDerivedType, MyDerivedType>&>(
                        value));
            } else {
                if constexpr (Index + 1 < sizeof...(DerivedTypes)) {
                    return as_variant_impl<BaseVariantTraitsType, IsConst, TupleType, Index + 1>::
                        exec(value, index);
                }
            }
        }
        throw std::runtime_error("Invalid Input");
        // This should never happen, just making a dummy to suppress warnings
        return RetType(
            std::in_place_type_t<FirstRefType>{},
            reinterpret_cast<
                std::conditional_t<IsConst, const FirstDerivedType, FirstDerivedType>&>(value));
    }
};
} // namespace detail

template <typename BaseVariantTraitsType>
typename BaseVariantTraitsType::ReferenceVariant as_variant(
    typename BaseVariantTraitsType::BaseType& value)
{
    using impl = detail::as_variant_impl<
        BaseVariantTraitsType,
        false,
        typename BaseVariantTraitsType::DerivedTypesTuple,
        0>;
    return impl::exec(value, BaseVariantTraitsType::get_index(value));
}

template <typename BaseVariantTraitsType>
typename BaseVariantTraitsType::ConstReferenceVariant as_const_variant(
    const typename BaseVariantTraitsType::BaseType& value)
{
    using impl = detail::as_variant_impl<
        BaseVariantTraitsType,
        true,
        typename BaseVariantTraitsType::DerivedTypesTuple,
        0>;
    return impl::exec(value, BaseVariantTraitsType::get_index(value));
}
} // namespace wmtk::utils::metaprogramming
