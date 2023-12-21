#include "TetMeshInvariant.hpp"
#include <wmtk/TetMesh.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
TetMeshInvariant::TetMeshInvariant(const TetMesh& mesh)
    : Invariant(mesh)
{}
const TetMesh& TetMeshInvariant::mesh() const
{
    return static_cast<const TetMesh&>(Invariant::mesh());
}
} // namespace wmtk
