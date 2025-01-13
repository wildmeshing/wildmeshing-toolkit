
#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::tests {
std::vector<PrimitiveType> primitives_up_to(PrimitiveType pt);


int64_t max_tuple_count(PrimitiveType pt);

Tuple tuple_from_offset_id(PrimitiveType pt, int offset);

std::vector<Tuple> all_valid_local_tuples(PrimitiveType pt, int64_t global_id = 0);
} // namespace wmtk::tests
