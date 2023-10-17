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
}

