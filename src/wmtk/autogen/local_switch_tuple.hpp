
#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

// NOTE: this header primarily exists to simplify unit testing, not really for use
namespace wmtk::autogen {
Tuple local_switch_tuple(PrimitiveType mesh_type, const Tuple& t, PrimitiveType pt);
}
