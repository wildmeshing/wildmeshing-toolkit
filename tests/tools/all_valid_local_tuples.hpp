
#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::tests {
std::vector<PrimitiveType> primitives_up_to(PrimitiveType pt);


long max_tuple_count(PrimitiveType pt);

Tuple tuple_from_offset_id(PrimitiveType pt, int offset);

std::vector<Tuple> all_valid_local_tuples(PrimitiveType pt);
} // namespace wmtk::tests
