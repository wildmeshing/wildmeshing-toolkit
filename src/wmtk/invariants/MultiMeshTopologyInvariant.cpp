#include "MultiMeshTopologyInvariant.hpp"

#include <stdexcept>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/MultiMeshSimplexVisitor.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk {
MultiMeshEdgeTopologyInvariant::MultiMeshEdgeTopologyInvariant(
    const Mesh& parent,
    const EdgeMesh& child)
    : Invariant(parent, true, false, false)
    , m_child_mesh(child)
{}

bool MultiMeshEdgeTopologyInvariant::before(const simplex::Simplex& t) const
{
    const Tuple v1 = t.tuple();
    const Tuple v2 = mesh().switch_tuple(t.tuple(), PrimitiveType::Vertex);

    // if the edge is in the child mesh, return true
    if (mesh()
            .map_to_child_tuples(m_child_mesh, simplex::Simplex(PrimitiveType::Edge, v1))
            .size() != 0)
        return true;

    // now the edge is not in the childmesh, then if both vertices are in the child mesh, return
    // false
    if (mesh().map_to_child_tuples(m_child_mesh, simplex::Simplex(PrimitiveType::Vertex, v1))
                .size() != 0 &&
        mesh().map_to_child_tuples(m_child_mesh, simplex::Simplex(PrimitiveType::Vertex, v2))
                .size() != 0) {
        return false;
    }


    return true;
}
} // namespace wmtk
