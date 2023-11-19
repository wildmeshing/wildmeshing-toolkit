#pragma once
#include <tuple>

#include "DerivedReferenceWrapperVariantTraits.hpp"
#include "tuple/as_variant_type.hpp"
#include "tuple/get_unique_remove_void_types.hpp"
#include "unwrap_ref.hpp"
namespace wmtk::utils::metaprogramming {

namespace detail {
// A helper class for specifying per-type return types from an input functor
// Assumes the argument is the variant type being selected form, all other
// arguments are passed in as const references
template <typename Functor, typename... Ts>
struct ReferenceWrappedFunctorReturnType
{
};
template <typename Functor, typename... VTs, typename... Ts>
struct ReferenceWrappedFunctorReturnType<Functor, std::tuple<VTs...>, Ts...>
{
    // For a specific type in the variant, get the return type
    template <typename T>
    using ReturnType =
        std::decay_t<std::invoke_result_t<Functor, unwrap_ref_decay_t<T>&, const Ts&...>>;

    template <typename T>
    using ConstReturnType =
        std::decay_t<std::invoke_result_t<Functor, const unwrap_ref_decay_t<T>&, const Ts&...>>;

    template <typename T>
    using ReturnTypeRef =
        std::conditional_t<std::is_same_v<ReturnType<T>, void>, void, ReturnType<T>&>;

    template <typename T>
    using ReturnTypeConstRef =
        std::conditional_t<std::is_same_v<ReturnType<T>, void>, void, const ReturnType<T>&>;

    // raw set of return values (might have duplicates or voids)
    using DirtyReturnTypesTuple = std::tuple<ReturnType<VTs>...>;

    // remove duplicates and any void return values
    using ReturnTypesTuple =
        tuple::get_unique_remove_void_types_from_tuple_t<DirtyReturnTypesTuple>;


    // if the return type as tuple is an empty tuple then there were no non-void return values
    constexpr static bool all_void = std::is_same_v<ReturnTypesTuple, std::tuple<>>;
    // Get an overall variant for the types
    using type = tuple::as_variant_type_t<ReturnTypesTuple>;
};
template <typename Functor, typename... VTs, typename... Ts>
struct ReferenceWrappedFunctorReturnType<Functor, std::variant<VTs...>, Ts...>
    : public ReferenceWrappedFunctorReturnType<Functor, std::tuple<VTs...>, Ts...>
{
};
} // namespace detail

template <typename Functor, typename ReferenceWrapperTraits, typename... Ts>
using ReferenceWrappedFunctorReturnType = typename detail::ReferenceWrappedFunctorReturnType<
    Functor,
    typename ReferenceWrapperTraits::ReferenceTuple,
    Ts...>::type;


} // namespace wmtk::utils::metaprogramming
