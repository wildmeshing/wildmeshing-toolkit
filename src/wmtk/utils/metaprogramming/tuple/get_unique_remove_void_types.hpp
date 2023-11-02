#pragma once
#include <tuple>
#include <type_traits>
#include "get_unique_types.hpp"
#include "remove_void_types.hpp"


namespace wmtk::utils::metaprogramming::tuple {


// given a set of types, a tuple without the voids and makes unique types
template <typename... Ts>
using get_unique_remove_void_types_t = // get_unique_t<Ts...>;
    typename detail::remove_void_tuple<void, get_unique_types_t<Ts...>>::type;


namespace detail {
template <typename T>
struct get_unique_remove_void_types_from_tuple
{
};
template <typename... Ts>
struct get_unique_remove_void_types_from_tuple<std::tuple<Ts...>>
{
    using type = get_unique_remove_void_types_t<Ts...>;
};

} // namespace detail

// from a tuple input
template <typename T>
using get_unique_remove_void_types_from_tuple_t =
    typename detail::get_unique_remove_void_types_from_tuple<T>::type;


} // namespace wmtk::utils::metaprogramming::tuple

