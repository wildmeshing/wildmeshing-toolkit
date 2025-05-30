
#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::tests {
std::vector<PrimitiveType> primitives_up_to(PrimitiveType pt);


int64_t max_tuple_count(PrimitiveType pt);

Tuple tuple_from_offset_id(PrimitiveType pt, int offset);

} // namespace wmtk::tests
