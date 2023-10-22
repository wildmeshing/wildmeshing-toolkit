#include "UpdateEdgeOperationMultiMeshMapFunctor.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <spdlog/spdlog.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/simplex/top_level_cofaces.hpp>
#include <wmtk/utils/TupleInspector.hpp>


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
    // spdlog::warn("{} {}", m.top_cell_dimension(), simplices_to_update.size());
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
{throw std::runtime_error("not implemented"); }

// tri -> edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TriMesh&,
    const tri_mesh::EdgeOperationData&,
    EdgeMesh&,
    const edge_mesh::EdgeOperationData&) const
{throw std::runtime_error("not implemented"); }
// tri -> tri
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TriMesh& parent_mesh,
    const tri_mesh::EdgeOperationData& parent_tmoe,
    TriMesh& child_mesh,
    const tri_mesh::EdgeOperationData& child_tmoe) const
{
    // spdlog::info(
    //     "UpdateEdgeOperationMultiMeshMapFunctor [{}=>{}] parent tmoe {} and child tmoe {}",
    //     fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
    //     fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
    //     wmtk::utils::TupleInspector::as_string(parent_tmoe.m_operating_tuple),
    //     wmtk::utils::TupleInspector::as_string(child_tmoe.m_operating_tuple));

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

    // spdlog::info(
    //     "Child had {} datas, parent had {} datas",
    //     child_incident_datas.size(),
    //     child_incident_datas.size());

    for (const auto& child_data : child_incident_datas) {
        long target_parent_fid = parent_global_cid(child_to_parent_accessor, child_data.fid);
        //    spdlog::info(
        //        "[{}=>{}] child data started with gid {}->{} and parent had {}",
        //        fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
        //        fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
        //        child_data.fid,
        //        fmt::join(child_data.split_f, ","),
        //        target_parent_fid);

        for (const auto& parent_data : parent_incident_datas) {
            // spdlog::info(
            //     "Check for parent fid {} to be {} ({})",
            //     parent_data.fid,
            //     target_parent_fid,
            //     parent_data.fid == target_parent_fid);
            if (parent_data.fid == target_parent_fid) {
                // if there was a parent mapping we definitely need to update the edges
                const auto& child_split_f = child_data.split_f;
                const auto& parent_split_f = parent_data.split_f;


                for (long index = 0; index < 2; ++index) {
                    long f_child = child_split_f[index];
                    long f_parent = parent_split_f[index];
                    if (f_child == -1 || f_parent == -1) {
                        continue;
                    }

                    long e_child = child_data.ears[index].eid;
                    long e_parent = parent_data.ears[index].eid;

                    long v_child = child_spine_v[index];
                    long v_parent = parent_spine_v[index];

                    const Tuple parent_tuple =
                        parent_mesh.tuple_from_global_ids(f_parent, e_parent, v_parent);
                    const Tuple child_tuple =
                        child_mesh.tuple_from_global_ids(f_child, e_child, v_child);
                    // spdlog::info(
                    //     "[{}=>{}] combining these setes of GIDS: Parent: {} {} {} {}; Child: {}
                    //     {} "
                    //     "{} {}",
                    //   fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
                    //   fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
                    //   f_parent,
                    //   e_parent,
                    //   v_parent,

                    //   wmtk::utils::TupleInspector::as_string(parent_tuple),
                    //   f_child,
                    //   e_child,
                    //   v_child,
                    //   wmtk::utils::TupleInspector::as_string(child_tuple));


                    assert(parent_mesh.is_valid_slow(parent_tuple));
                    assert(child_mesh.is_valid_slow(child_tuple));

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
    // spdlog::info(
    //    "[{0}=>{1}] updating hashes for {0}",
    //    fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
    //    fmt::join(child_mesh.absolute_multi_mesh_id(), ","));
    update_all_hashes(parent_mesh, parent_tmoe.global_simplex_ids_with_potentially_modified_hashes);
    // spdlog::info(
    //    "[{0}=>{1}] updating hashes for {1}",
    //    fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
    //    fmt::join(child_mesh.absolute_multi_mesh_id(), ","));

    update_all_hashes(child_mesh, child_tmoe.global_simplex_ids_with_potentially_modified_hashes);
}

// tet -> edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh&,
    const tet_mesh::EdgeOperationData&,
    EdgeMesh&,
    const edge_mesh::EdgeOperationData&) const
{throw std::runtime_error("not implemented"); }
// tet -> tri
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh&,
    const tet_mesh::EdgeOperationData&,
    TriMesh&,
    const tri_mesh::EdgeOperationData&) const
{throw std::runtime_error("not implemented"); }
// tet -> tet
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh&,
    const tet_mesh::EdgeOperationData&,
    TetMesh&,
    const tet_mesh::EdgeOperationData&) const
{throw std::runtime_error("not implemented"); }


long UpdateEdgeOperationMultiMeshMapFunctor::child_global_cid(
    const attribute::ConstAccessor<long>& parent_to_child,
    long parent_gid) const
{
    return MultiMeshManager::child_global_cid(parent_to_child, parent_gid);
}
long UpdateEdgeOperationMultiMeshMapFunctor::parent_global_cid(
    const attribute::ConstAccessor<long>& child_to_parent,
    long child_gid) const
{
    return MultiMeshManager::parent_global_cid(child_to_parent, child_gid);
}

} // namespace wmtk::operations::utils
