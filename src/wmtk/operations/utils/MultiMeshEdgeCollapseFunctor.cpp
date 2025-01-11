
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
    return std::move(static_cast<edge_mesh::EdgeOperationData&>(exec));
}
tri_mesh::EdgeOperationData MultiMeshEdgeCollapseFunctor::operator()(
    TriMesh& m,
    const simplex::Simplex& s) const
{
    assert(m.is_valid(s));
    TriMesh::TriMeshOperationExecutor exec(m, s.tuple());
#if defined(MTAO_CONSTANTLY_VERIFY_MESH)
    assert(m.is_connectivity_valid());
#endif
    exec.collapse_edge();
#if defined(MTAO_CONSTANTLY_VERIFY_MESH)
    assert(m.is_connectivity_valid());
#endif
    return std::move(static_cast<tri_mesh::EdgeOperationData&>(exec));
}
tet_mesh::EdgeOperationData MultiMeshEdgeCollapseFunctor::operator()(
    TetMesh& m,
    const simplex::Simplex& s) const
{
    TetMesh::TetMeshOperationExecutor exec(m, s.tuple());
    exec.collapse_edge();
    return std::move(static_cast<tet_mesh::EdgeOperationData&>(exec));
}
} // namespace wmtk::operations::utils
