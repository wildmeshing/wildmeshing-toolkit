
#include "is_ccw.hpp"
#include <cassert>
#include <wmtk/utils/TupleInspector.hpp>

namespace wmtk::autogen::edge_mesh {
bool is_ccw(const Tuple& tuple)
{
    assert(tuple_is_valid_for_ccw(tuple));
    using namespace utils;
    return TupleInspector::local_vid(tuple) == 0;
}
bool tuple_is_valid_for_ccw(const Tuple& tuple)
{
    return true;
}
} // namespace wmtk::autogen::edge_mesh
