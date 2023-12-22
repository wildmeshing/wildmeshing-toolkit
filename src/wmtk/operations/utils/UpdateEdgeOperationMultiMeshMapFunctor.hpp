#pragma once
#include <memory>
#include <vector>

namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
class Tuple;
namespace attribute {
template <typename T>
class ConstAccessor;
}

namespace operations::data {
class EdgeMeshEdgeOperationData;
class TriMeshEdgeOperationData;
class TetMeshEdgeOperationData;
} // namespace operations::data

namespace operations::utils {

class UpdateEdgeOperationMultiMeshMapFunctor
{
public:
    // edge -> edge
    void operator()(
        EdgeMesh&,
        const data::EdgeMeshEdgeOperationData& parent_tmoe,
        EdgeMesh&,
        const data::EdgeMeshEdgeOperationData&) const;

    // tri -> edge
    void operator()(
        TriMesh&,
        const data::TriMeshEdgeOperationData&,
        EdgeMesh&,
        const data::EdgeMeshEdgeOperationData&) const;
    // tri -> tri
    void operator()(
        TriMesh&,
        const data::TriMeshEdgeOperationData&,
        TriMesh&,
        const data::TriMeshEdgeOperationData&) const;

    // tet -> edge
    void operator()(
        TetMesh&,
        const data::TetMeshEdgeOperationData&,
        EdgeMesh&,
        const data::EdgeMeshEdgeOperationData&) const;
    // tet -> tri
    void operator()(
        TetMesh&,
        const data::TetMeshEdgeOperationData&,
        TriMesh&,
        const data::TriMeshEdgeOperationData&) const;
    // tet -> tet
    void operator()(
        TetMesh&,
        const data::TetMeshEdgeOperationData&,
        TetMesh&,
        const data::TetMeshEdgeOperationData&) const;

    // edge
    void operator()(EdgeMesh&, const data::EdgeMeshEdgeOperationData& parent_tmoe) const;

    // tri
    void operator()(TriMesh&, const data::TriMeshEdgeOperationData&);

    // tet
    void operator()(TetMesh&, const data::TetMeshEdgeOperationData&);

private:
    long parent_global_cid(const attribute::ConstAccessor<long>& parent_to_child, long parent_gid)
        const;
    long child_global_cid(const attribute::ConstAccessor<long>& parent_to_child, long parent_gid)
        const;
    void update_all_hashes(
        Mesh& m,
        const std::vector<std::vector<std::tuple<long, std::vector<Tuple>>>>& simplices_to_update,
        const std::vector<std::tuple<long, std::array<long, 2>>>& split_cell_maps = {}) const;
    void update_ear_replacement(TriMesh& m, const data::TriMeshEdgeOperationData& fmoe) const;
    // TODO: add tet version
};
} // namespace operations::utils
} // namespace wmtk
