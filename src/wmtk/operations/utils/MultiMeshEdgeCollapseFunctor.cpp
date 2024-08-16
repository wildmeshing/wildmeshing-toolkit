
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
    EdgeMesh::EdgeMeshOperationExecutor exec(m, s.tuple());
    exec.collapse_edge();
    return exec;
}
tri_mesh::EdgeOperationData MultiMeshEdgeCollapseFunctor::operator()(
    TriMesh& m,
    const simplex::Simplex& s) const
{
    TriMesh::TriMeshOperationExecutor exec(m, s.tuple());
    exec.collapse_edge();

    return exec;
}
tet_mesh::EdgeOperationData MultiMeshEdgeCollapseFunctor::operator()(
    TetMesh& m,
    const simplex::Simplex& s) const
{
    TetMesh::TetMeshOperationExecutor exec(m, s.tuple());
    exec.collapse_edge();
    return exec;
}
} // namespace wmtk::operations::utils
