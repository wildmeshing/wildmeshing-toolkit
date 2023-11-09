
#include "MultiMeshEdgeCollapseFunctor.hpp"
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/operations/Operation.hpp>

namespace wmtk::operations::utils {

void MultiMeshEdgeCollapseFunctor::operator()(const Mesh&, const Simplex&) const
{
    throw std::runtime_error("Unimplemented!");
}

edge_mesh::EdgeOperationData MultiMeshEdgeCollapseFunctor::operator()(
    EdgeMesh& m,
    const simplex::Simplex& s) const
{
    Accessor<long> hash_accessor = Operation::get_hash_accessor(m);
    EdgeMesh::EdgeMeshOperationExecutor exec(m, s.tuple(), hash_accessor);
    exec.collapse_edge();
    return exec;
}
tri_mesh::EdgeOperationData MultiMeshEdgeCollapseFunctor::operator()(TriMesh& m, const Simplex& s)
    const
{
    Accessor<long> hash_accessor = Operation::get_hash_accessor(m);
    TriMesh::TriMeshOperationExecutor exec(m, s.tuple(), hash_accessor);
    exec.collapse_edge();

    return exec;
}
tet_mesh::EdgeOperationData MultiMeshEdgeCollapseFunctor::operator()(TetMesh& m, const Simplex& s)
    const
{
    Accessor<long> hash_accessor = Operation::get_hash_accessor(m);
    TetMesh::TetMeshOperationExecutor exec(m, s.tuple(), hash_accessor);
    exec.collapse_edge();
    return exec;
}
}; // namespace wmtk::operations::utils
