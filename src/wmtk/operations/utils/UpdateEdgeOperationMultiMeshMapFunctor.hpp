#pragma once
#include <memory>
#include <vector>

namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
namespace attribute {
template <typename T>
class ConstAccessor;
}

namespace operations {
namespace edge_mesh {
class EdgeOperationData;
}
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
    // edge -> edge
    void operator()(
        EdgeMesh&,
        const edge_mesh::EdgeOperationData& parent_tmoe,
        EdgeMesh&,
        const edge_mesh::EdgeOperationData&) const;

    // tri -> edge
    void operator()(
        TriMesh&,
        const tri_mesh::EdgeOperationData&,
        EdgeMesh&,
        const edge_mesh::EdgeOperationData&) const;
    // tri -> tri
    void operator()(
        TriMesh&,
        const tri_mesh::EdgeOperationData&,
        TriMesh&,
        const tri_mesh::EdgeOperationData&) const;

    // tet -> edge
    void operator()(
        TetMesh&,
        const tet_mesh::EdgeOperationData&,
        EdgeMesh&,
        const edge_mesh::EdgeOperationData&) const;
    // tet -> tri
    void operator()(
        TetMesh&,
        const tet_mesh::EdgeOperationData&,
        TriMesh&,
        const tri_mesh::EdgeOperationData&) const;
    // tet -> tet
    void operator()(
        TetMesh&,
        const tet_mesh::EdgeOperationData&,
        TetMesh&,
        const tet_mesh::EdgeOperationData&) const;

private:
    long child_global_cid(const attribute::ConstAccessor<long>& parent_to_child, long parent_gid)
        const;
};
} // namespace operations::utils
} // namespace wmtk
