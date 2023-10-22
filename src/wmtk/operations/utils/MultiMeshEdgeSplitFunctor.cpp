#include "MultiMeshEdgeSplitFunctor.hpp"
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/operations/Operation.hpp>
#include <wmtk/utils/TupleInspector.hpp>

namespace wmtk::operations::utils {

void MultiMeshEdgeSplitFunctor::operator()(const Mesh&, const Simplex&) const
{
    throw "Unimplemented!";
}

edge_mesh::EdgeOperationData MultiMeshEdgeSplitFunctor::operator()(
    EdgeMesh& m,
    const simplex::Simplex& s) const
{
    Accessor<long> hash_accessor = Operation::get_hash_accessor(m);
    EdgeMesh::EdgeMeshOperationExecutor exec(m, s.tuple(), hash_accessor);
    exec.split_edge();
    return exec;
}
tri_mesh::EdgeOperationData MultiMeshEdgeSplitFunctor::operator()(TriMesh& m, const Simplex& s)
    const
{
    spdlog::info(
        "[{}] Splitting with tuple {}",
        fmt::join(m.absolute_multi_mesh_id(), ","),
        wmtk::utils::TupleInspector::as_string(s.tuple()));
    Accessor<long> hash_accessor = Operation::get_hash_accessor(m);
    TriMesh::TriMeshOperationExecutor exec(m, s.tuple(), hash_accessor);
    exec.split_edge();
    return exec;
}
tet_mesh::EdgeOperationData MultiMeshEdgeSplitFunctor::operator()(TetMesh& m, const Simplex& s)
    const
{
    Accessor<long> hash_accessor = Operation::get_hash_accessor(m);
    TetMesh::TetMeshOperationExecutor exec(m, s.tuple(), hash_accessor);
    exec.split_edge();
    return exec;
}
}; // namespace wmtk::operations::utils
