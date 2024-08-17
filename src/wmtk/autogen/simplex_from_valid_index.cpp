#include "simplex_from_valid_index.hpp"
#include <cassert>
#include <wmtk/autogen/edge_mesh/simplex_from_valid_index.hpp>
#include <wmtk/autogen/tet_mesh/simplex_from_valid_index.hpp>
#include <wmtk/autogen/tri_mesh/simplex_from_valid_index.hpp>
namespace wmtk::autogen {
int8_t simplex_from_valid_index(
    PrimitiveType mesh_type,
    int8_t valid_tuple_index,
    PrimitiveType simplex_type)
{
    switch (mesh_type) {
    case PrimitiveType::Tetrahedron:
        return tet_mesh::simplex_from_valid_index(valid_tuple_index, simplex_type);
    case PrimitiveType::Triangle:
        return tri_mesh::simplex_from_valid_index(valid_tuple_index, simplex_type);
    case PrimitiveType::Edge:
        return edge_mesh::simplex_from_valid_index(valid_tuple_index, simplex_type);
    case PrimitiveType::Vertex:
    default: assert(false); // "not implemented"
    }
    return {};
}
} // namespace wmtk::autogen
