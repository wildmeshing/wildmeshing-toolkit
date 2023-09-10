#include "TriMeshInvariant.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
TriMeshInvariant::TriMeshInvariant(const TriMesh& mesh)
    : MeshInvariant(mesh)
{}
const TriMesh& TriMeshInvariant::mesh() const
{
    return static_cast<const TriMesh&>(MeshInvariant::mesh());
}
} // namespace wmtk
