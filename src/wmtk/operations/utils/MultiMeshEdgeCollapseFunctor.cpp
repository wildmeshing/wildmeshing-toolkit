
#include "MultiMeshEdgeCollapseFunctor.hpp"
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/operations/Operation.hpp>

namespace wmtk::operations::utils {

void MultiMeshEdgeCollapseFunctor::operator()(const Mesh&, const simplex::Simplex&) const
{
    throw std::runtime_error("Unimplemented!");
}

edge_mesh::EdgeOperationData MultiMeshEdgeCollapseFunctor::operator()(
    EdgeMesh& m,
    const simplex::Simplex& s) const
{
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();
    EdgeMesh::EdgeMeshOperationExecutor exec(m, s.tuple(), hash_accessor);
    exec.collapse_edge();
    return exec;
}
tri_mesh::EdgeOperationData MultiMeshEdgeCollapseFunctor::operator()(
    TriMesh& m,
    const simplex::Simplex& s) const
{
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();
    TriMesh::TriMeshOperationExecutor exec(m, s.tuple(), hash_accessor);
    exec.collapse_edge();

    return exec;
}
tet_mesh::EdgeOperationData MultiMeshEdgeCollapseFunctor::operator()(
    TetMesh& m,
    const simplex::Simplex& s) const
{
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();
    TetMesh::TetMeshOperationExecutor exec(m, s.tuple(), hash_accessor);
    exec.collapse_edge();
    return exec;
}
}; // namespace wmtk::operations::utils
