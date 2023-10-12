#include "EdgeMeshOperation.hpp"
#include <wmtk/EdgeMesh.hpp>

namespace wmtk::operations::edge_mesh {
EdgeMeshOperation::EdgeMeshOperation(EdgeMesh& m)
    : m_mesh(m)
    , m_hash_accessor(get_hash_accessor(m))
{}

EdgeMeshOperation::EdgeMeshOperation(Mesh& m)
    : EdgeMeshOperation(dynamic_cast<EdgeMesh&>(m))
{}
Mesh& EdgeMeshOperation::base_mesh() const
{
    return m_mesh;
}
Accessor<long>& EdgeMeshOperation::hash_accessor()
{
    return m_hash_accessor;
}

EdgeMesh& EdgeMeshOperation::mesh() const
{
    return m_mesh;
}
} // namespace wmtk::operations::edge_mesh
