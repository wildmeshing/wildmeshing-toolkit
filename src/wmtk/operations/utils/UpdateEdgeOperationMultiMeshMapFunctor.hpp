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
namespace simplex {
class Simplex;
}
namespace attribute {
template <typename T, typename MeshType>
class Accessor;
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

class UpdateEdgeOperationMultiMeshMapFunctor
{
public:
    // edge -> edge
    [[noreturn]] void operator()(
        EdgeMesh&,
        const simplex::Simplex&,
        const edge_mesh::EdgeOperationData& parent_tmoe,
        EdgeMesh&,
        const simplex::Simplex&,
        const edge_mesh::EdgeOperationData&) const;

    // tri -> edge
    void operator()(
        TriMesh&,
        const simplex::Simplex&,
        const tri_mesh::EdgeOperationData&,
        EdgeMesh&,
        const simplex::Simplex&,
        const edge_mesh::EdgeOperationData&) const;
    // tri -> tri
    void operator()(
        TriMesh&,
        const simplex::Simplex&,
        const tri_mesh::EdgeOperationData&,
        TriMesh&,
        const simplex::Simplex&,
        const tri_mesh::EdgeOperationData&) const;

    // tet -> edge
    void operator()(
        TetMesh&,
        const simplex::Simplex&,
        const tet_mesh::EdgeOperationData&,
        EdgeMesh&,
        const simplex::Simplex&,
        const edge_mesh::EdgeOperationData&) const;
    // tet -> tri
    void operator()(
        TetMesh&,
        const simplex::Simplex&,
        const tet_mesh::EdgeOperationData&,
        TriMesh&,
        const simplex::Simplex&,
        const tri_mesh::EdgeOperationData&) const;
    // tet -> tet
    void operator()(
        TetMesh&,
        const simplex::Simplex&,
        const tet_mesh::EdgeOperationData&,
        TetMesh&,
        const simplex::Simplex&,
        const tet_mesh::EdgeOperationData&) const;

    // edge
    void operator()(EdgeMesh&, const simplex::Simplex&, const edge_mesh::EdgeOperationData&);

    // tri
    void operator()(TriMesh&, const simplex::Simplex&, const tri_mesh::EdgeOperationData&);

    // tet
    void operator()(TetMesh&, const simplex::Simplex&, const tet_mesh::EdgeOperationData&);

private:
    int64_t parent_global_cid(
        const attribute::Accessor<int64_t,Mesh>& parent_to_child,
        int64_t parent_gid) const;
    int64_t child_global_cid(
        const attribute::Accessor<int64_t,Mesh>& parent_to_child,
        int64_t parent_gid) const;
    void update_all_hashes(
        Mesh& m,
        const std::vector<std::vector<std::tuple<int64_t, std::vector<Tuple>>>>&
            simplices_to_update,
        const std::vector<std::tuple<int64_t, std::array<int64_t, 2>>>& split_cell_maps = {}) const;
    void update_ear_replacement(TriMesh& m, const tri_mesh::EdgeOperationData& fmoe) const;

    // for tet
    int64_t parent_local_fid(
        const attribute::Accessor<int64_t,Mesh>& parent_to_child,
        int64_t parent_gid) const;

    void update_ear_replacement(TetMesh& m, const tet_mesh::EdgeOperationData& tmoe) const;
};
} // namespace operations::utils
} // namespace wmtk
