#pragma once

#include <vector>
#include <wmtk/Primitive.hpp>
namespace wmtk::utils {
// returns a vector of primitives including the endpoints of the range
std::vector<PrimitiveType> primitive_range(PrimitiveType pt0, PrimitiveType pt1);
// returns a vector of primitives including the endpoint
std::vector<PrimitiveType> primitive_above(PrimitiveType pt0, bool lower_to_upper = true);
// returns a vector of primitives including the endpoint
std::vector<PrimitiveType> primitive_below(PrimitiveType pt1, bool lower_to_upper = false);
} // namespace wmtk::utils
