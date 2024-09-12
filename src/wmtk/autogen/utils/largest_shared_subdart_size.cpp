#include "subdart_maximal_action_to_face.hpp"
#include <cassert>
#include <wmtk/autogen/edge_mesh/subdart_maximal_action_to_face.hpp>
#include <wmtk/autogen/tet_mesh/subdart_maximal_action_to_face.hpp>
#include <wmtk/autogen/tri_mesh/subdart_maximal_action_to_face.hpp>

namespace wmtk::autogen::utils {
auto largest_shared_subdart_size(
    PrimitiveType mesh_type,
    int8_t dart_index,
    int8_t simplex_dimension,
    int8_t simplex_index) -> int8_t
{
    switch (mesh_type) {
    case PrimitiveType::Edge:
        return edge_mesh::subdart_maximal_action_to_face_size(
            dart_index,
            simplex_dimension,
            simplex_index);
    case PrimitiveType::Triangle:
        return tri_mesh::subdart_maximal_action_to_face_size(
            dart_index,
            simplex_dimension,
            simplex_index);
    case PrimitiveType::Tetrahedron:
        return tet_mesh::subdart_maximal_action_to_face_size(
            dart_index,
            simplex_dimension,
            simplex_index);
    case PrimitiveType::Vertex: return {};
    default: assert(false);
    }
    return {};
}

int8_t largest_shared_subdart_size(
    PrimitiveType mesh_type,
    int8_t dart_index,
    PrimitiveType primitive_type,
    int8_t simplex_index)
{
    const int8_t type_index = get_primitive_type_id(primitive_type);
    return largest_shared_subdart_size(mesh_type, dart_index, type_index, simplex_index);
}
} // namespace wmtk::autogen::utils
