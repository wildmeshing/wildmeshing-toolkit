#include "local_switch_tuple.hpp"
#include <wmtk/autogen/tet_mesh/local_switch_tuple.hpp>
#include <wmtk/autogen/tri_mesh/local_switch_tuple.hpp>
namespace wmtk::autogen {
Tuple local_switch_tuple(PrimitiveType mesh_type, const Tuple& t, PrimitiveType pt)
{
    switch (mesh_type) {
    case PrimitiveType::Face: return tri_mesh::local_switch_tuple(t, pt);
    case PrimitiveType::Tetrahedron: return tet_mesh::local_switch_tuple(t, pt);
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge: throw "notimplemented"; 
    }
    return Tuple();
}
} // namespace wmtk::autogen
