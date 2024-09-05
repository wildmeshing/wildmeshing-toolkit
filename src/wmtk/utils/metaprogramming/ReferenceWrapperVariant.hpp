#pragma once
#include <functional>
#include <type_traits>
#include <utility>
#include <variant>

namespace wmtk::utils::metaprogramming {

// My target application's "Input" class is quite heavy and the Input objects
// persist for int64_t periods of time relative to what this is being used for, so
// I want to use a variant of references rather than values
//
// Here's a helper definition for making variants of references
template <typename... T>
using ReferenceWrapperVariant = std::variant<std::reference_wrapper<T>...>;


} // namespace wmtk::utils::metaprogramming
