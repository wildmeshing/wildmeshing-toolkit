#include "MultiMeshEdgeSplitFunctor.hpp"

#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/operations/Operation.hpp>

#include <wmtk/utils/Logger.hpp>

namespace wmtk::operations::utils {

wmtk::operations::EdgeOperationData MultiMeshEdgeSplitFunctor::run(
    Mesh& mesh,
    const simplex::Simplex& s) const
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Vertex: break;
    case PrimitiveType::Edge: return (*this)(static_cast<EdgeMesh&>(mesh), s);
    case PrimitiveType::Triangle: return (*this)(static_cast<TriMesh&>(mesh), s);
    case PrimitiveType::Tetrahedron: return (*this)(static_cast<TetMesh&>(mesh), s);
    default: break;
    }
    assert(false);
    return {};
}

edge_mesh::EdgeOperationData MultiMeshEdgeSplitFunctor::operator()(
    EdgeMesh& m,
    const simplex::Simplex& s) const
{
    EdgeMesh::EdgeMeshOperationExecutor exec(m, s.tuple());
    exec.split_edge();
    return std::move(static_cast<edge_mesh::EdgeOperationData&>(exec));
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

    return std::move(static_cast<tri_mesh::EdgeOperationData&>(exec));
}
tet_mesh::EdgeOperationData MultiMeshEdgeSplitFunctor::operator()(
    TetMesh& m,
    const simplex::Simplex& s) const
{
    TetMesh::TetMeshOperationExecutor exec(m, s.tuple());
    exec.split_edge();
    return std::move(static_cast<tet_mesh::EdgeOperationData&>(exec));
}
} // namespace wmtk::operations::utils
