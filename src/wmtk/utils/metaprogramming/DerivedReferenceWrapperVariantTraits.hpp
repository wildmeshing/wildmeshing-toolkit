#pragma once
#include <tuple>

#include "ReferenceWrapperVariant.hpp"
#include "tuple/as_variant_type.hpp"
#include "tuple/concatenate_types.hpp"
namespace wmtk::utils::metaprogramming {
template <typename BaseType_, typename... DerivedTypes>
struct DerivedReferenceWrapperVariantTraits
{
    using BaseType = BaseType_;

    // we keep a plain derived type tuple for convenience
    using DerivedTypesTuple = std::tuple<DerivedTypes...>;
    // we keep reference types of derived type tuple for convenience
    using ReferenceTuple = std::tuple<std::reference_wrapper<DerivedTypes>...>;
    using ConstReferenceTuple = std::tuple<std::reference_wrapper<const DerivedTypes>...>;

    using AllReferenceTuple = tuple::concatenate_types_t<ReferenceTuple, ConstReferenceTuple>;

    // The reference class for this type
    using ReferenceVariant = tuple::as_variant_type_t<ReferenceTuple>;
    using ConstReferenceVariant = tuple::as_variant_type_t<ConstReferenceTuple>;
    using AllReferenceVariant = tuple::as_variant_type_t<AllReferenceTuple>;

    // convenience functions for getting ref variants with constness

    template <bool IsConst>
    using ReferenceTuple_const = std::conditional_t<IsConst, ConstReferenceTuple, ReferenceTuple>;
    template <bool IsConst>
    using ReferenceVariant_const =
        std::conditional_t<IsConst, ConstReferenceVariant, ReferenceVariant>;

    static size_t get_index(const BaseType& t);
};

} // namespace wmtk::utils::metaprogramming
