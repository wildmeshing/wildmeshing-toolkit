#include "local_switch_tuple.hpp"
#include <wmtk/Tuple.hpp>
#include <wmtk/autogen/edge_mesh/local_switch_tuple.hpp>
#include <wmtk/autogen/tet_mesh/local_switch_tuple.hpp>
#include <wmtk/autogen/tri_mesh/local_switch_tuple.hpp>
namespace wmtk::multimesh::utils {

Tuple local_switch_tuple(
    PrimitiveType mesh_primitive_type,
    const Tuple& source,
    PrimitiveType primitive_type)
{
    switch (mesh_primitive_type) {
    case PrimitiveType::Face: return autogen::tri_mesh::local_switch_tuple(source, primitive_type);
    case PrimitiveType::Tetrahedron:
        return autogen::tet_mesh::local_switch_tuple(source, primitive_type);
    case PrimitiveType::Edge: return autogen::edge_mesh::local_switch_tuple(source, primitive_type);
    case PrimitiveType::Vertex:
    default: return Tuple();
    }
}

Tuple local_switch_tuples(
    PrimitiveType mesh_primitive_type,
    const Tuple& tuple,
    const std::initializer_list<PrimitiveType>& op_sequence)
{
    return local_switch_tuples<std::initializer_list<PrimitiveType>>(
        mesh_primitive_type,
        tuple,
        op_sequence);
}
} // namespace wmtk::multimesh::utils
