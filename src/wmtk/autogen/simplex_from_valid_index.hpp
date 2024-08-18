#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

// NOTE: this header primarily exists to simplify unit testing, not really for use
namespace wmtk::autogen {
int8_t simplex_from_valid_index(
    PrimitiveType mesh_type,
    int8_t valid_tuple_index,
    PrimitiveType simplex_type);

} // namespace wmtk::autogen

