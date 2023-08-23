#include "Operation.hpp"
#include <wmtk/TetMesh.hpp>

namespace wmtk::operations::tet_mesh {
Operation::Operation(Mesh& m)
    : wmtk::operations::Operation(m)
{}

TetMesh& Operation::mesh() const
{
    return static_cast<TetMesh&>(m_mesh);
}
} // namespace wmtk::operations::tet_mesh
