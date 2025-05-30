#include "FusionEdgeInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {
FusionEdgeInvariant::FusionEdgeInvariant(const Mesh& position_mesh, const Mesh& periodic_mesh)
    : Invariant(position_mesh)
    , m_periodic_mesh(periodic_mesh)
{}

bool FusionEdgeInvariant::before(const simplex::Simplex& s) const
{
    throw("removing multimesh");

    // if (!mesh().is_boundary(PrimitiveType::Edge, s.tuple())) {
    //    // non boundary edge, check boundary vertrex
    //    // if any one of the vertex is on boundary, return false
    //    if (mesh().is_boundary(PrimitiveType::Vertex, s.tuple()) ||
    //        mesh().is_boundary(
    //            PrimitiveType::Vertex,
    //            mesh().switch_tuple(s.tuple(), PrimitiveType::Vertex))) {
    //        return false;
    //    } else {
    //        return true;
    //    }
    //} else {
    //    std::array<wmtk::simplex::Simplex, 2> periodic_v = {
    //        {mesh().map_to_parent(wmtk::simplex::Simplex::vertex(mesh(), s.tuple())),
    //         mesh().map_to_parent(wmtk::simplex::Simplex::vertex(
    //             mesh(),
    //             mesh().switch_tuple(s.tuple(), PrimitiveType::Vertex)))}};
    //
    //    // check periodic vertex is on the the periodic mesh boundary, if is, return false
    //    // only allow operation on edges that are not on the boudnary of the periodic mesh
    //    if (m_periodic_mesh.is_boundary(PrimitiveType::Vertex, periodic_v[0].tuple()) ||
    //        m_periodic_mesh.is_boundary(PrimitiveType::Vertex, periodic_v[1].tuple())) {
    //        return false;
    //    }
    //
    //    // check periodic vertex can be mapped to more than 2 vertieces in position mesh
    //    // check corner
    //    if (m_periodic_mesh.map_to_child(mesh(), periodic_v[0]).size() > 2 ||
    //        m_periodic_mesh.map_to_child(mesh(), periodic_v[1]).size() > 2) {
    //        return false;
    //    }
    //
    //    return true;
    //}

    // TODO: add code for 3d
}


} // namespace wmtk::invariants
