#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::autogen::tet_mesh {
Tuple local_switch_tuple(const Tuple& t, PrimitiveType pt);

Tuple local_switch_tuple(const Tuple& t, int8_t valid_tuple_index);
}

namespace internal {
    int8_t switch_primitive_to_valid_tuple_index(wmtk::PrimitiveType pt);
    int8_t identity_valid_tuple_index();
}

#include "local_switch_tuple.hxx"
