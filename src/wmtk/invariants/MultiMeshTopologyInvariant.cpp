#include "MultiMeshTopologyInvariant.hpp"

#include <stdexcept>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk {
MultiMeshEdgeTopologyInvariant::MultiMeshEdgeTopologyInvariant(
    const Mesh& parent,
    const EdgeMesh& child)
    : Invariant(parent, true, false, false)
// , m_child_mesh(child)
{}

bool MultiMeshEdgeTopologyInvariant::before(const simplex::Simplex& t) const
{
    throw("removed due to removal of multimesh");
    return true;
}
} // namespace wmtk
