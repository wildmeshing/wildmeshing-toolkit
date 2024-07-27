
#include "tuple_from_valid_index.hpp"
#include <cassert>
#include <wmtk/autogen/edge_mesh/tuple_from_valid_index.hpp>
#include <wmtk/autogen/tet_mesh/tuple_from_valid_index.hpp>
#include <wmtk/autogen/tri_mesh/tuple_from_valid_index.hpp>
namespace wmtk::autogen {
Tuple tuple_from_valid_index(
    PrimitiveType mesh_type,
    const int64_t global_cid,
    int8_t valid_tuple_index)
{
    switch (mesh_type) {
    case PrimitiveType::Tetrahedron:
        return tet_mesh::tuple_from_valid_index(global_cid, valid_tuple_index);
    case PrimitiveType::Triangle:
        return tri_mesh::tuple_from_valid_index(global_cid, valid_tuple_index);
    case PrimitiveType::Edge:
        return edge_mesh::tuple_from_valid_index(global_cid, valid_tuple_index);
    case PrimitiveType::Vertex: return Tuple(-1, -1, -1, global_cid);
    default: assert(false); // "not implemented"
    }
    return {};
}
} // namespace wmtk::autogen
