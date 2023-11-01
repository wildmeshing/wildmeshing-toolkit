
#pragma once
#include <functional>
#include <type_traits>
#include <utility>
#include <variant>


namespace wmtk::utils::metaprogramming {
// Re-implement unwrap ref in case it doesn't exist with our current compiler
// Implementation is the Possible Implementation from:
// https://en.cppreference.com/w/cpp/utility/functional/unwrap_reference

template <class T>
struct unwrap_reference
{
    using type = T;
};
template <class U>
struct unwrap_reference<std::reference_wrapper<U>>
{
    using type = U&;
};

template <class T>
struct unwrap_ref_decay : unwrap_reference<std::decay_t<T>>
{
};

template <class T>
using unwrap_ref_decay_t = typename unwrap_ref_decay<T>::type;
} // namespace wmtk::utils::metaprogramming
