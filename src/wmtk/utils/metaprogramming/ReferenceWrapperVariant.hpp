#include <functional>
#include <type_traits>
#include <utility>
#include <variant>

#pragma once
// Re-implement unwrap ref in case it doesn't exist with our current compiler
// Implementation is the Possible Implementation from:
// https://en.cppreference.com/w/cpp/utility/functional/unwrap_reference
#if !defined(__cpp_lib_unwrap_ref)
namespace std {

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
struct unwrap_ref_decay : std::unwrap_reference<std::decay_t<T>>
{
};

template <class T>
using unwrap_ref_decay_t = typename unwrap_ref_decay<T>::type;
} // namespace std
#endif
namespace wmtk::utils::metaprogramming {

// My target application's "Input" class is quite heavy and the Input objects
// persist for long periods of time relative to what this is being used for, so
// I want to use a variant of references rather than values
//
// Here's a helper definition for making variants of references
template <typename... T>
using ReferenceWrapperVariant = std::variant<std::reference_wrapper<T>...>;


} // namespace wmtk::utils::metaprogramming
