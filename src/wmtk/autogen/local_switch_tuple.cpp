#include "local_switch_tuple.hpp"
#include <wmtk/autogen/edge_mesh/local_switch_tuple.hpp>
#include <wmtk/autogen/tet_mesh/local_switch_tuple.hpp>
#include <wmtk/autogen/tri_mesh/local_switch_tuple.hpp>
namespace wmtk::autogen {
Tuple local_switch_tuple(PrimitiveType mesh_type, const Tuple& t, PrimitiveType pt)
{
    switch (mesh_type) {
    case PrimitiveType::Triangle: return tri_mesh::local_switch_tuple(t, pt);
    case PrimitiveType::Tetrahedron: return tet_mesh::local_switch_tuple(t, pt);
    case PrimitiveType::Edge: return edge_mesh::local_switch_tuple(t, pt);
    case PrimitiveType::Vertex:
    default: assert(false); // "not implemented"
    }
    return {};
}
Tuple local_switch_tuple(PrimitiveType mesh_type, const Tuple& t, int8_t valid_tuple_index)
{
    switch (mesh_type) {
    case PrimitiveType::Triangle: return tri_mesh::local_switch_tuple(t, valid_tuple_index);
    case PrimitiveType::Tetrahedron: return tet_mesh::local_switch_tuple(t, valid_tuple_index);
    case PrimitiveType::Edge: return edge_mesh::local_switch_tuple(t, valid_tuple_index);
    case PrimitiveType::Vertex:
    default: assert(false); // "not implemented"
    }
    return {};
}

namespace internal {
int8_t switch_primitive_to_valid_tuple_index(PrimitiveType mesh_type, PrimitiveType pt)
{
    switch (mesh_type) {
    case PrimitiveType::Tetrahedron:
        return tet_mesh::internal::switch_primitive_to_valid_tuple_index(pt);
    case PrimitiveType::Triangle:
        return tri_mesh::internal::switch_primitive_to_valid_tuple_index(pt);
    case PrimitiveType::Edge: return edge_mesh::internal::switch_primitive_to_valid_tuple_index(pt);
    case PrimitiveType::Vertex:
    default: assert(false); // "not implemented"
    }
    return {};
}
int8_t identity_valid_tuple_index(PrimitiveType mesh_type)
{
    switch (mesh_type) {
    case PrimitiveType::Tetrahedron: return tet_mesh::internal::identity_valid_tuple_index();
    case PrimitiveType::Triangle: return tri_mesh::internal::identity_valid_tuple_index();
    case PrimitiveType::Edge: return edge_mesh::internal::identity_valid_tuple_index();
    case PrimitiveType::Vertex:
    default: assert(false); // "not implemented"
    }
    return {};
}
} // namespace internal
} // namespace wmtk::autogen
