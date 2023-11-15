#include "MultiMeshTopologyInvariant.hpp"
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk {
MultiMeshEdgeTopologyInvariant::MultiMeshEdgeTopologyInvariant(
    const Mesh& parent,
    const Mesh& child,
    const PrimitiveType pt)
    : MeshInvariant(parent)
    , m_child_mesh(child)
    , m_primitive_type(pt)
{}

bool MultiMeshEdgeTopologyInvariant::before(const Tuple& t) const
{
    assert(m_child_mesh.top_simplex_type() == PrimitiveType::Edge);
    const Tuple v1 = t;
    const Tuple v2 = mesh().switch_vertex(t);

    // if less or equal than one vertex is in the child mesh, return true
    if (mesh().map_to_child_tuples(m_child_mesh, Simplex(PrimitiveType::Vertex, v1)).size() == 0 ||
        mesh().map_to_child_tuples(m_child_mesh, Simplex(PrimitiveType::Vertex, v2)).size() == 0)
        return true;

    // if both vertices are in the child mesh, and the edge is not in the child mesh, return false
    if (mesh().map_to_child_tuples(m_child_mesh, Simplex(PrimitiveType::Edge, t)).size() == 0)
        return false;
    else
        return true;
}
} // namespace wmtk