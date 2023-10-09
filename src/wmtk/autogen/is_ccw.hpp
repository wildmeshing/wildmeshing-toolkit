#pragma once
#include <wmtk/Primitive.hpp>

namespace wmtk {
class Tuple;
}

// NOTE: this header primarily exists to simplify unit testing, not really for use
namespace wmtk::autogen {
bool is_ccw(PrimitiveType ptype, const Tuple& t);

// validates whether the tuple local ids are valid for computing ccw'ness
bool tuple_is_valid_for_ccw(PrimitiveType ptype, const Tuple& t);
} // namespace wmtk::autogen
