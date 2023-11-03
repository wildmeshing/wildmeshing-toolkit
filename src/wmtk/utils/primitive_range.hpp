#pragma once

#include <vector>
#include <wmtk/Primitive.hpp>
namespace wmtk::utils {
// returns a vector of primitives including the endpoitns of the range
std::vector<PrimitiveType> primitive_range(PrimitiveType pt0, PrimitiveType pt1);
// returns a vector of primitives including the endpoint
std::vector<PrimitiveType> primitive_above(PrimitiveType pt0);
// returns a vector of primitives including the endpoint
std::vector<PrimitiveType> primitive_below(PrimitiveType pt1);
} // namespace wmtk::utils
