#include "TriMeshOperation.hpp"
#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::tri_mesh {
TriMeshOperation::TriMeshOperation(Mesh& m)
    : Operation(m)
    , m_mesh(dynamic_cast<TriMesh&>(m))
{}

} // namespace wmtk::operations::tri_mesh
