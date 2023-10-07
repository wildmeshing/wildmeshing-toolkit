#pragma once
#include <memory>

namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;

namespace operations {
class EdgeMeshOperationExecutor;
namespace tri_mesh {
class EdgeOperationData;
}
namespace tet_mesh {
class EdgeOperationData;
}
} // namespace operations

namespace operations::utils {

struct UpdateEdgeOperationMultiMeshMapFunctor
{
    void operator()(
        TriMesh& parent_mesh,
        const tri_mesh::EdgeOperationData& parent_tmoe,
        TriMesh& child_mesh,
        const tri_mesh::EdgeOperationData& child_tmoe) const;
    void operator()(
        EdgeMesh&,
        const EdgeMeshOperationExecutor&,
        EdgeMesh&,
        const EdgeMeshOperationExecutor&) const;
    void operator()(
        TriMesh&,
        const tri_mesh::EdgeOperationData&,
        EdgeMesh&,
        const EdgeMeshOperationExecutor&) const;
    void operator()(
        TetMesh&,
        const tet_mesh::EdgeOperationData&,
        TriMesh&,
        const tri_mesh::EdgeOperationData&) const;
};
} // namespace operations::utils
} // namespace wmtk
