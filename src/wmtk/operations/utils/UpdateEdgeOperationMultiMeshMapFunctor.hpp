#pragma once
#include <memory>

namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;

class EdgeMeshOperationExecutor;
class TriMeshOperationExecutor;
class TetMeshOperationExecutor;

namespace operations::utils {

struct UpdateEdgeOperationMultiMeshMapFunctor
{
    void operator()(
        TriMesh& parent_mesh,
        const TriMeshOperationExecutor& parent_tmoe,
        TriMesh& child_mesh,
        const TriMeshOperationExecutor& child_tmoe);
    void operator()(
        EdgeMesh&,
        const EdgeMeshOperationExecutor&,
        EdgeMesh&,
        EdgeMeshOperationExecutor&) const;
    void operator()(
        TriMesh&,
        const TriMeshOperationExecutor&,
        EdgeMesh&,
        const EdgeMeshOperationExecutor&) const;
    void operator()(
        TetMesh&,
        const TetMeshOperationExecutor&,
        TriMesh&,
        const TriMeshOperationExecutor&) const;
};
} // namespace operations::utils
} // namespace wmtk
