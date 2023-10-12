#pragma once
#include <tuple>

#include "DerivedReferenceWrapperVariantTraits.hpp"
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
        std::decay_t<std::invoke_result_t<Functor, std::unwrap_ref_decay_t<T>&, const Ts&...>>;

    // Get an overall variant for the types
    using type = std::variant<ReturnType<VTs>...>;
};
} // namespace detail

template <typename Functor, typename ReferenceWrapperTraits, typename... Ts>
using ReferenceWrappedFunctorReturnType = detail::ReferenceWrappedFunctorReturnType<
    Functor,
    typename ReferenceWrapperTraits::ReferenceTuple,
    Ts...>;

} // namespace wmtk::utils::metaprogramming
