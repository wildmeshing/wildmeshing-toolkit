#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::autogen::tet_mesh {
Tuple local_switch_tuple(const Tuple& t, PrimitiveType pt);

Tuple local_switch_tuple(const Tuple& t, int8_t valid_tuple_index);
}

#include "local_switch_tuple.hxx"
