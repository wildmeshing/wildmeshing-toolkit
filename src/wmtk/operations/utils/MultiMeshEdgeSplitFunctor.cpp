#include "MultiMeshEdgeSplitFunctor.hpp"
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/operations/Operation.hpp>

namespace wmtk::operations::utils {

void MultiMeshEdgeSplitFunctor::operator()(const Mesh&, const Simplex&) const
{
    throw "Unimplemented!";
}
tri_mesh::EdgeOperationData MultiMeshEdgeSplitFunctor::operator()(TriMesh& m, const Simplex& s)
    const
{
    Accessor<long> hash_accessor = Operation::get_hash_accessor(m);
    TriMesh::TriMeshOperationExecutor exec(m, t, hash_accessor);
    exec.split_edge();
    return exec;
}
tri_mesh::EdgeOperationData MultiMeshEdgeSplitFunctor::operator()(TetMesh& m, const Simplex& s)
    const
{
    Accessor<long> hash_accessor = Operation::get_hash_accessor(m);
    TetMesh::TetMeshOperationExecutor exec(m, t, hash_accessor);
    exec.split_edge();
    return exec;
}
}; // namespace wmtk::operations::utils
