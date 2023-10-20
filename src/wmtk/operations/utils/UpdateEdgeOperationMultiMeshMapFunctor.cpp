#include "UpdateEdgeOperationMultiMeshMapFunctor.hpp"
//#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/simplex/top_level_cofaces.hpp>


namespace wmtk::operations::utils {

namespace {
constexpr static PrimitiveType PV = PrimitiveType::Vertex;
constexpr static PrimitiveType PE = PrimitiveType::Edge;
constexpr static PrimitiveType PF = PrimitiveType::Face;
constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;
} // namespace
void UpdateEdgeOperationMultiMeshMapFunctor::update_all_hashes(
    Mesh& m,
    const std::vector<std::vector<std::tuple<long, std::vector<Tuple>>>>& simplices_to_update,
    const std::vector<std::tuple<long, std::array<long, 2>>>& split_cell_maps) const
{
    assert(m.top_cell_dimension() + 1 == simplices_to_update.size());
    constexpr static PrimitiveType PTs[] = {PV, PE, PF, PT};
    for (size_t j = 0; j < simplices_to_update.size(); ++j) {
        m.m_multi_mesh_manager
            .update_map_tuple_hashes(m, PTs[j], simplices_to_update[j], split_cell_maps);
    }
}


// edge -> edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    EdgeMesh&,
    const edge_mesh::EdgeOperationData& parent_tmoe,
    EdgeMesh&,
    const edge_mesh::EdgeOperationData&) const
{}

// tri -> edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TriMesh&,
    const tri_mesh::EdgeOperationData&,
    EdgeMesh&,
    const edge_mesh::EdgeOperationData&) const
{}
// tri -> tri
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TriMesh& parent_mesh,
    const tri_mesh::EdgeOperationData& parent_tmoe,
    TriMesh& child_mesh,
    const tri_mesh::EdgeOperationData& child_tmoe) const
{
    const auto& parent_incident_datas = parent_tmoe.incident_face_datas();
    const auto& child_incident_datas = child_tmoe.incident_face_datas();

    const auto& child_spine_v = child_tmoe.incident_vids();
    const auto& parent_spine_v = parent_tmoe.incident_vids();


    auto& parent_mmmanager = parent_mesh.m_multi_mesh_manager;
    auto& child_mmmanager = child_mesh.m_multi_mesh_manager;

    auto child_to_parent_handle = child_mmmanager.map_to_parent_handle;

    long child_id = child_mmmanager.child_id();
    auto parent_to_child_handle = parent_mmmanager.children().at(child_id).map_handle;
    auto child_to_parent_accessor = child_mesh.create_accessor(child_to_parent_handle);
    auto parent_to_child_accessor = parent_mesh.create_accessor(parent_to_child_handle);

    using IncidentData = tri_mesh::EdgeOperationData::IncidentFaceData;

    for (const auto& parent_data : parent_incident_datas) {
        long target_child_fid = child_global_cid(parent_to_child_accessor, parent_data.fid);

        for (const auto& child_data : child_incident_datas) {
            if (child_data.fid == target_child_fid) {
                // if there was a parent mapping we definitely need to update the edges
                const auto& child_split_f = child_data.split_f;
                const auto& parent_split_f = parent_data.split_f;


                for (long index = 0; index < 2; ++index) {
                    long f_child = child_split_f[index];
                    long f_parent = parent_split_f[index];

                    long e_child = child_data.ears[index].eid;
                    long e_parent = parent_data.ears[index].eid;

                    long v_child = child_spine_v[index];
                    long v_parent = parent_spine_v[index];
                    spdlog::info(
                        "Parent GIDs: {} {} {}; Child GIDs: {} {} {}",
                        f_parent,
                        e_parent,
                        v_parent,
                        f_child,
                        e_child,
                        v_child);

                    const Tuple parent_tuple =
                        parent_mesh.tuple_from_global_ids(f_parent, e_parent, v_parent);
                    const Tuple child_tuple =
                        child_mesh.tuple_from_global_ids(f_child, e_child, v_child);

                    spdlog::info("{} => {}", 
                            wmtk::utils::TupleInspector::as_string(parent_tuple),
                            wmtk::utils::TupleInspector::as_string(child_tuple));

                    wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                        parent_to_child_accessor,
                        child_to_parent_accessor,
                        parent_tuple,
                        child_tuple);
                }
            }
        }
    }

    // update_hash on neighboring cells. use only 2 to get the cell types on either case
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Face;

    // NOTE: this is purpuosely verbose to show a point
    // We have to select with PrimitiveTypes are supported as children for each type of mesh
    //
    update_all_hashes(parent_mesh, parent_tmoe.global_simplex_ids_with_potentially_modified_hashes);

    update_all_hashes(child_mesh, child_tmoe.global_simplex_ids_with_potentially_modified_hashes);
}

// tet -> edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh&,
    const tet_mesh::EdgeOperationData&,
    EdgeMesh&,
    const edge_mesh::EdgeOperationData&) const
{}
// tet -> tri
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh&,
    const tet_mesh::EdgeOperationData&,
    TriMesh&,
    const tri_mesh::EdgeOperationData&) const
{}
// tet -> tet
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh&,
    const tet_mesh::EdgeOperationData&,
    TetMesh&,
    const tet_mesh::EdgeOperationData&) const
{}


long UpdateEdgeOperationMultiMeshMapFunctor::child_global_cid(
    const attribute::ConstAccessor<long>& parent_to_child,
    long parent_gid) const
{
    return MultiMeshManager::child_global_cid(parent_to_child, parent_gid);
}

} // namespace wmtk::operations::utils
