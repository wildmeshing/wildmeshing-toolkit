#include "uvEdgeInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {
uvEdgeInvariant::uvEdgeInvariant(const Mesh& position_mesh, const Mesh& uv_mesh)
    : Invariant(position_mesh)
    , m_uv_mesh(uv_mesh)
{}

bool uvEdgeInvariant::before(const simplex::Simplex& s) const
{
    throw("commenting out because removing multimesh");
    // if (s.primitive_type() == PrimitiveType::Vertex) {
    //     auto uv_v = mesh().map_to_child(m_uv_mesh, s);
    //     for (const auto& v : uv_v) {
    //         if (m_uv_mesh.is_boundary(PrimitiveType::Vertex, v.tuple())) return false;
    //     }
    //     return true;
    // }
    //
    // else if (s.primitive_type() == PrimitiveType::Edge) {
    //     auto uv_v1 = mesh().map_to_child(m_uv_mesh, simplex::Simplex::vertex(mesh(), s.tuple()));
    //     auto uv_v2 = mesh().map_to_child(
    //         m_uv_mesh,
    //         simplex::Simplex::vertex(
    //             mesh(),
    //             mesh().switch_tuple(s.tuple(), PrimitiveType::Vertex)));
    //
    //     for (const auto& v : uv_v1) {
    //         if (m_uv_mesh.is_boundary(PrimitiveType::Vertex, v.tuple())) return false;
    //     }
    //     for (const auto& v : uv_v2) {
    //         if (m_uv_mesh.is_boundary(PrimitiveType::Vertex, v.tuple())) return false;
    //     }
    //
    //     return true;
    // }

    return true;
}


} // namespace wmtk::invariants
