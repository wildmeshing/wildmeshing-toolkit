#include "MultiMeshEdgeSplitFunctor.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/operations/Operation.hpp>

namespace wmtk::operations::utils {

void MultiMeshEdgeSplitFunctor::operator()(const Mesh&, const Simplex&) const
{
    throw std::runtime_error("Unimplemented!");
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
    Accessor<long> hash_accessor = Operation::get_hash_accessor(m);
    TriMesh::TriMeshOperationExecutor exec(m, s.tuple(), hash_accessor);

    exec.split_edge();
    for (const auto& id : exec.incident_face_datas()) {
        spdlog::info(
            "[{}] mapped {}->{}",
            fmt::join(m.absolute_multi_mesh_id(), ","),
            id.fid,
            fmt::join(id.split_f, ","));
    }

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
