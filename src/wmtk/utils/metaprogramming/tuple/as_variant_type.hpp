#pragma once
#include <tuple>
#include <variant>


namespace wmtk::utils::metaprogramming::tuple {


// given a set of types, a tuple without the voids and makes unique types
namespace detail {
template <typename T>
struct as_variant_type
{
};
template <typename... Ts>
struct as_variant_type<std::tuple<Ts...>>
{
    using type = std::variant<Ts...>;
};

} // namespace detail

// from a tuple input
template <typename T>
using as_variant_type_t = typename detail::as_variant_type<T>::type;


} // namespace wmtk::utils::metaprogramming::tuple

