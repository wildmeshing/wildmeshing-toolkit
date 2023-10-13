#include "is_ccw.hpp"
#include <wmtk/Tuple.hpp>
#include <wmtk/autogen/tet_mesh/is_ccw.hpp>
#include <wmtk/autogen/tri_mesh/is_ccw.hpp>
namespace wmtk::autogen {
bool is_ccw(PrimitiveType pt, const Tuple& t)
{
    switch (pt) {
    case PrimitiveType::Face: return tri_mesh::is_ccw(t);
    case PrimitiveType::Tetrahedron: return tet_mesh::is_ccw(t);
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
    default: throw "notimplemented";
    }
    return false;
}

// validates whether the tuple local ids are valid for computing ccw'ness
bool tuple_is_valid_for_ccw(PrimitiveType pt, const Tuple& t)
{
    switch (pt) {
    case PrimitiveType::Face: return tri_mesh::tuple_is_valid_for_ccw(t);
    case PrimitiveType::Tetrahedron: return tet_mesh::tuple_is_valid_for_ccw(t);
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
    default: throw "notimplemented";
    }
    return false;
}
} // namespace wmtk::autogen
