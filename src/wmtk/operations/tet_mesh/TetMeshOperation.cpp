#include "TetMeshOperation.hpp"
#include <wmtk/TetMesh.hpp>

namespace wmtk::operations::tet_mesh {
TetMeshOperation::TetMeshOperation(TetMesh& m)
    : m_mesh(m)
    , m_hash_accessor(get_hash_accessor(m))
{}

TetMeshOperation::TetMeshOperation(Mesh& m)
    : TetMeshOperation(dynamic_cast<TetMesh&>(m))
{}
Mesh& TetMeshOperation::base_mesh() const
{
    return m_mesh;
}
Accessor<long>& TetMeshOperation::hash_accessor()
{
    return m_hash_accessor;
}

TetMesh& TetMeshOperation::mesh() const
{
    return m_mesh;
}
} // namespace wmtk::operations::tet_mesh
