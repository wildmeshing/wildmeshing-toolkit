#include "MultiMeshEdgeSplitFunctor.hpp"

#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/operations/Operation.hpp>

#include <wmtk/utils/Logger.hpp>

namespace wmtk::operations::utils {

void MultiMeshEdgeSplitFunctor::operator()(const Mesh&, const simplex::Simplex&) const
{
    throw std::runtime_error("Unimplemented!");
}

edge_mesh::EdgeOperationData MultiMeshEdgeSplitFunctor::operator()(
    EdgeMesh& m,
    const simplex::Simplex& s) const
{
    EdgeMesh::EdgeMeshOperationExecutor exec(m, s.tuple());
    exec.split_edge();
    return exec;
}
tri_mesh::EdgeOperationData MultiMeshEdgeSplitFunctor::operator()(
    TriMesh& m,
    const simplex::Simplex& s) const
{
    TriMesh::TriMeshOperationExecutor exec(m, s.tuple());

    exec.split_edge();
    for (const auto& id : exec.incident_face_datas()) {
        logger().trace(
            "[{}] mapped {}->{}",
            fmt::join(m.absolute_multi_mesh_id(), ","),
            id.fid,
            fmt::join(id.split_f, ","));
    }

    return exec;
}
tet_mesh::EdgeOperationData MultiMeshEdgeSplitFunctor::operator()(
    TetMesh& m,
    const simplex::Simplex& s) const
{
    TetMesh::TetMeshOperationExecutor exec(m, s.tuple());
    exec.split_edge();
    return exec;
}
} // namespace wmtk::operations::utils
