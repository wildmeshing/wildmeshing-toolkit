#pragma once
namespace wmtk {
class Tuple;
}

namespace wmtk::autogen::tet_mesh {
bool is_ccw(const Tuple& t);

// validates whether the tuple local ids are valid for computing ccw'ness
bool tuple_is_valid_for_ccw(const Tuple& t);
} // namespace wmtk::autogen::tet_mesh

#include "is_ccw.hxx"