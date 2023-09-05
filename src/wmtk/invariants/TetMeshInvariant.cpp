#include "TetMeshInvariant.hpp"
#include <wmtk/TetMesh.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
TetMeshInvariant::TetMeshInvariant(const TetMesh& mesh)
    : MeshInvariant(mesh)
{}
const TetMesh& TetMeshInvariant::mesh() const
{
    return static_cast<const TetMesh&>(MeshInvariant::mesh());
}
} // namespace wmtk
