
#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

// NOTE: this header primarily exists to simplify unit testing, not really for use
namespace wmtk::autogen {
Tuple local_switch_tuple(PrimitiveType mesh_type, const Tuple& t, PrimitiveType pt);
Tuple local_switch_tuple(PrimitiveType mesh_type, const Tuple& t, int8_t valid_tuple_index);

namespace internal {
int8_t switch_primitive_to_valid_tuple_index(PrimitiveType mesh_type, PrimitiveType pt);
int8_t identity_valid_tuple_index(PrimitiveType mesh_type);
} // namespace internal
} // namespace wmtk::autogen
