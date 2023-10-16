#pragma once
#include <tuple>
#include "concatenate_types.hpp"


namespace wmtk::utils::metaprogramming::tuple {
namespace detail {
template <typename...>
struct remove_void
{
    using type = std::tuple<>;
};
template <>
struct remove_void<void>
{
    using type = std::tuple<>;
};
template <typename T>
struct remove_void<T>
{
    using type = std::tuple<T>;
};

template <typename... Ts>
using remove_void_t = typename remove_void<Ts...>::type;

template <typename T, typename RemainingTuple>
struct remove_void_tuple
{
};
template <typename T, typename... Ts>
struct remove_void_tuple<T, std::tuple<Ts...>>
{
    constexpr static bool t_not_void = !std::is_same_v<T, void>;
    using adding_t_type = std::conditional_t<t_not_void, std::tuple<T>, std::tuple<>>;

    using type = concatenate_types_t<adding_t_type, remove_void_t<Ts...>>;
};

template <typename T, typename... Ts>
struct remove_void<T, Ts...>
{
    using type = typename remove_void_tuple<T, std::tuple<Ts...>>::type;
};
} // namespace detail

template <typename... Ts>
using remove_void_types_t = detail::remove_void_t<Ts...>;
} // namespace wmtk::utils::metaprogramming::tuple
