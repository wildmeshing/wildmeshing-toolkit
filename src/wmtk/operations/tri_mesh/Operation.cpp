#include "Operation.hpp"
#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::tri_mesh {
Operation::Operation(Mesh& m)
    : wmtk::operations::Operation(m)
{}

TriMesh& Operation::mesh() const
{
    return dynamic_cast<TriMesh&>(m_mesh);
}
} // namespace wmtk::operations::tri_mesh
