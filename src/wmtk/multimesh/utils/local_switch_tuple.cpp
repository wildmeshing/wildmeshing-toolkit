#include "local_switch_tuple.hpp"
#include <wmtk/Tuple.hpp>
#include <wmtk/autogen/tet_mesh/local_switch_tuple.hpp>
#include <wmtk/autogen/tri_mesh/local_switch_tuple.hpp>

namespace wmtk::multimesh::utils {

Tuple local_switch_tuple(const Tuple& source, PrimitiveType primitive_type)
{
    switch (primitive_type) {
    case PrimitiveType::Face: return autogen::tri_mesh::local_switch_tuple(source);
    case PrimitiveType::Tetrahedron: return autogen::tet_mesh::local_switch_tuple(source);
    default: return Tuple();
    }

    Tuple local_switch_tuples(
        PrimitiveType mesh_primitive_type,
        const Tuple& tuple,
        const std::initializer_list<PrimitiveType>& op_sequence) const
    {
        return local_switch_tuples<std::initializer_list<PrimitiveType>>(
            mesh_primitive_type,
            tuple,
            op_sequence);
    }
}
