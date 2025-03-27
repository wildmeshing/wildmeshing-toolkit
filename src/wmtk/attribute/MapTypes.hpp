#pragma once
#include <Eigen/Core>
#ifndef WMTK_MAX_ATTRIBUTE_DIMENSION
#define WMTK_MAX_ATTRIBUTE_DIMENSION Eigen::Dynamic
#endif

namespace wmtk::attribute {
namespace internal {

/// this value is set by CMake for the max size an attribute should hold
/// any time we write an attribute to a vector we should use this type
constexpr static Eigen::Index MAX_ATTR_SIZE = WMTK_MAX_ATTRIBUTE_DIMENSION;

/// Underlying vector type used by attribute is dynamically sized but bounded
template <typename T, int R = Eigen::Dynamic>
using VectorType = Eigen::Matrix<T, R, 1, 0, (R == Eigen::Dynamic ? MAX_ATTR_SIZE : R), 1>;
}

/// the default map type used by attributes is a map of our vector type.
/// Though the max size doesn't affect the storage of the map, this affects the
/// type returned by .eval()
template <typename T, int R = Eigen::Dynamic>
using MapResult = typename internal::VectorType<T, R>::MapType;
template <typename T, int R = Eigen::Dynamic>
using ConstMapResult = typename internal::VectorType<T, R>::ConstMapType;

} // namespace wmtk::attribute::internal
