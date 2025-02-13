
#pragma once

#include <cassert>

namespace wmtk::autogen::edge_mesh {
inline bool is_ccw(const Tuple& tuple)
{
    assert(tuple_is_valid_for_ccw(tuple));
    return tuple.local_vid() == 0;
}
inline bool tuple_is_valid_for_ccw(const Tuple& tuple)
{
    if (tuple.is_null()) {
        return false;
    }
    return true;
}
} // namespace wmtk::autogen::edge_mesh
