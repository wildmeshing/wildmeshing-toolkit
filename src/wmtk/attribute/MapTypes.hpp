#pragma once
#include "internal/MapTypes.hpp"

namespace wmtk::attribute {

/// the default map type used by attributes is a map of our vector type.
/// Though the max size doesn't affect the storage of the map, this affects the
/// type returned by .eval()
template <typename T, int R = Eigen::Dynamic>
using MapResult = typename internal::VectorResult<T, R>::MapType;
template <typename T, int R = Eigen::Dynamic>
using ConstMapResult = typename internal::VectorResult<T, R>::ConstMapType;


} // namespace wmtk::attribute::internal
