#pragma once
#include <tuple>
#include <type_traits>
#include "concatenate_types.hpp"

namespace wmtk::utils::metaprogramming::tuple {


// given a tuple, extracts a tuple with only unique types in the input

// strategy is to recursively extract the last copy of any given type


// alternates between get_unique_types<tuple<Ts...> with
// main_unique_tuple_types<T,std::tuple<Ts...>> to combine Ts... and T,Ts... as necessary

namespace detail {


// basic initializations - by default return empty in case the input is empty (which is not caught
// by later cases)
template <typename...>
struct get_unique_types
{
    using type = std::tuple<>;
};
// basic initialization, expects a T and a Tuple
template <typename T, typename RemainingTuple>
struct get_unique_types_tuple
{
};
// convenience to avoid typename for get_unique_types...
template <typename... Ts>
using get_unique_types_t = typename get_unique_types<Ts...>::type;


// actual base case - single type is unique so we return that type
template <typename T>
struct get_unique_types<T>
{
    using type = std::tuple<T>;
};


// from a list of types, check if the first is unique compared to the rest
template <typename T, typename... Ts>
struct get_unique_types<T, Ts...>
{
    using type = typename get_unique_types_tuple<T, std::tuple<Ts...>>::type;
};


// do the meat of the implementation - check if T is unique in T,Ts...
// if so then add it to the unique of the rest
template <typename T, typename... Ts>
struct get_unique_types_tuple<T, std::tuple<Ts...>>
{
    // is t unique among T,Ts...
    constexpr static bool t_is_unique = (!std::is_same_v<T, Ts> && ...);
    // if its unique then create {T} otherwise {}
    using adding_t_type = std::conditional_t<t_is_unique, std::tuple<T>, std::tuple<>>;

    // concatenate adding_t_type \cup unique(Ts...)
    using type = concatenate_types_t<adding_t_type, get_unique_types_t<Ts...>>;
};


} // namespace detail

// main access point
template <typename... Ts>
using get_unique_types_t = detail::get_unique_types_t<Ts...>;

} // namespace wmtk::utils::metaprogramming::tuple
