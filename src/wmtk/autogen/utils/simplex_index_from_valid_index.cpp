#include "simplex_index_from_valid_index.hpp"
#include <wmtk/autogen/edge_mesh/simplex_index_from_valid_index.hpp>
#include <wmtk/autogen/tet_mesh/simplex_index_from_valid_index.hpp>
#include <wmtk/autogen/tri_mesh/simplex_index_from_valid_index.hpp>

namespace wmtk::autogen::utils {
auto simplex_index_from_valid_index(
    const PrimitiveType mesh_type,
    int8_t valid_index,
    wmtk::PrimitiveType type) -> int8_t
{
    switch (mesh_type) {
    case PrimitiveType::Edge: return edge_mesh::simplex_index_from_valid_index(valid_index, type);
    case PrimitiveType::Triangle:
        return tri_mesh::simplex_index_from_valid_index(valid_index, type);
    case PrimitiveType::Tetrahedron:
        return tet_mesh::simplex_index_from_valid_index(valid_index, type);
    case PrimitiveType::Vertex: return 0;
    default: assert(false);
    }
    return 0;
}
} // namespace wmtk::autogen::utils
