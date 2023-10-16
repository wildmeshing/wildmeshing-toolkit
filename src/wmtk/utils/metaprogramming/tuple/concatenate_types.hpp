#pragma once
#include <tuple>

namespace wmtk::utils::metaprogramming::tuple {


// creates a tuple that concatenates the types passed in


// generic case so we can specialize with tuples passed in
template <typename T, typename U>
struct concatenate_types
{
};

// main case where we pass in two tuples
template <typename... Ts, typename... Us>
struct concatenate_types<std::tuple<Ts...>, std::tuple<Us...>>
{
    using type = std::tuple<Ts..., Us...>;
};


// alias to underlying type
template <typename T, typename U>
using concatenate_types_t = typename concatenate_types<T, U>::type;
} // namespace wmtk::utils::metaprogramming::tuple
