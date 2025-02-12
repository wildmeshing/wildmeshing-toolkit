#pragma once
#include "internal/VectorTypes.hpp"

namespace wmtk::attribute {

/// the default map type used by attributes is a map of our vector type.
/// Though the max size doesn't affect the storage of the map, this affects the
/// type returned by .eval()
template <typename T, int R = Eigen::Dynamic>
using MapResult = internal::VectorMapType<T, R>;
template <typename T, int R = Eigen::Dynamic>
using ConstMapResult = internal::ConstVectorMapType<T, R>;


} // namespace wmtk::attribute::internal
