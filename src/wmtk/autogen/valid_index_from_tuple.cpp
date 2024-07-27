
#include "valid_index_from_tuple.hpp"
#include <cassert>
#include <wmtk/autogen/edge_mesh/valid_index_from_tuple.hpp>
#include <wmtk/autogen/tet_mesh/valid_index_from_tuple.hpp>
#include <wmtk/autogen/tri_mesh/valid_index_from_tuple.hpp>
namespace wmtk::autogen {
int8_t valid_index_from_tuple(PrimitiveType mesh_type, const Tuple& t)
{
    switch (mesh_type) {
    case PrimitiveType::Tetrahedron: return tet_mesh::valid_index_from_tuple(t);
    case PrimitiveType::Triangle: return tri_mesh::valid_index_from_tuple(t);
    case PrimitiveType::Edge: return edge_mesh::valid_index_from_tuple(t);
    case PrimitiveType::Vertex:
    default: return 0;
    }
    return {};
}
} // namespace wmtk::autogen
