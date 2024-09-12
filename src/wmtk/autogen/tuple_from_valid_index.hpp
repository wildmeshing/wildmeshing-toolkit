#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

// NOTE: this header primarily exists to simplify unit testing, not really for use
namespace wmtk::autogen {
Tuple tuple_from_valid_index(
    PrimitiveType mesh_type,
    const int64_t global_cid,
    int8_t valid_tuple_index);

} // namespace wmtk::autogen
 
