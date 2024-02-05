#pragma once
#include <Eigen/Core>
#ifndef WMTK_MAX_ATTRIBUTE_DIMENSION
#define WMTK_MAX_ATTRIBUTE_DIMENSION Eigen::Dynamic
#endif

namespace wmtk::attribute::internal {

// this value is set by CMake
constexpr static int MAX_ATTR_SIZE = WMTK_MAX_ATTRIBUTE_DIMENSION;
template <typename T, int R = Eigen::Dynamic>
using VectorResult = Eigen::Matrix<T, R, 1, 0, (R == Eigen::Dynamic ? MAX_ATTR_SIZE : R), 1>;

template <typename T, int R = Eigen::Dynamic>
using MapResult = typename VectorResult<T, R>::MapType;
template <typename T, int R = Eigen::Dynamic>
using ConstMapResult = typename VectorResult<T, R>::ConstMapType;

} // namespace wmtk::attribute::internal
