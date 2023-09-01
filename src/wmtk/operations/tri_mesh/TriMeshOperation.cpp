#include "TriMeshOperation.hpp"
#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::tri_mesh {
TriMeshOperation::TriMeshOperation(TriMesh& m)
    : m_mesh(m)
    , m_hash_accessor(get_hash_accessor(m))
{}
TriMeshOperation::TriMeshOperation(Mesh& m)
    : TriMeshOperation(dynamic_cast<TriMesh&>(m))
{}
Accessor<long>& TriMeshOperation::hash_accessor()
{
    return m_hash_accessor;
}
Mesh& TriMeshOperation::base_mesh() const
{
    return m_mesh;
}

TriMesh& TriMeshOperation::mesh() const
{
    return m_mesh;
}
} // namespace wmtk::operations::tri_mesh
