#include "local_id_table_offset.hpp"
#include <cassert>
#include <wmtk/autogen/edge_mesh/local_id_table_offset.hpp>
#include <wmtk/autogen/tet_mesh/local_id_table_offset.hpp>
#include <wmtk/autogen/tri_mesh/local_id_table_offset.hpp>

namespace wmtk::autogen::utils {
auto local_id_table_offset(PrimitiveType mesh_type, const Tuple& t) -> int8_t
{
    switch (mesh_type) {
    case PrimitiveType::Edge: return edge_mesh::local_id_table_offset(t);
    case PrimitiveType::Triangle: return tri_mesh::local_id_table_offset(t);
    case PrimitiveType::Tetrahedron: return tet_mesh::local_id_table_offset(t);
    case PrimitiveType::Vertex: return 0;
    default: assert(false);
    }
    return 0;
}
} // namespace wmtk::autogen::utils
