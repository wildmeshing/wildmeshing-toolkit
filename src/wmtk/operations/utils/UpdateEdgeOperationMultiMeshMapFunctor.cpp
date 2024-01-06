#include "UpdateEdgeOperationMultiMeshMapFunctor.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/TupleInspector.hpp>

#include <wmtk/utils/Logger.hpp>


namespace wmtk::operations::utils {

namespace {
constexpr static PrimitiveType PV = PrimitiveType::Vertex;
constexpr static PrimitiveType PE = PrimitiveType::Edge;
constexpr static PrimitiveType PF = PrimitiveType::Face;
constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;
} // namespace
void UpdateEdgeOperationMultiMeshMapFunctor::update_all_hashes(
    Mesh& m,
    const std::vector<std::vector<std::tuple<int64_t, std::vector<Tuple>>>>& simplices_to_update,
    const std::vector<std::tuple<int64_t, std::array<int64_t, 2>>>& split_cell_maps) const
{
    assert(m.top_cell_dimension() + 1 == simplices_to_update.size());
    constexpr static PrimitiveType PTs[] = {PV, PE, PF, PT};
    // logger().trace("{} {}", m.top_cell_dimension(), simplices_to_update.size());
    for (size_t j = 0; j < simplices_to_update.size(); ++j) {
        m.m_multi_mesh_manager
            .update_map_tuple_hashes(m, PTs[j], simplices_to_update[j], split_cell_maps);
    }
}

void UpdateEdgeOperationMultiMeshMapFunctor::update_ear_replacement(
    TriMesh& m,
    const tri_mesh::EdgeOperationData& fmoe) const
{
    const auto& parent_incident_datas = fmoe.incident_face_datas();
    auto& parent_mmmanager = m.m_multi_mesh_manager;
    auto parent_hash_accessor = m.get_const_cell_hash_accessor();

    for (int64_t index = 0; index < parent_incident_datas.size(); ++index) {
        const auto& ears = parent_incident_datas[index].ears;
        for (int64_t ear_index = 0; ear_index < 2; ++ear_index) {
            const int64_t ear_fid = ears[ear_index].fid;
            const int64_t ear_eid = ears[ear_index].eid;
            const int64_t ear_vid = fmoe.m_spine_vids[ear_index];

            const int64_t ear_fid_other = ears[1 - ear_index].fid;

            if (ear_fid != -1) continue; // safe

            for (auto child_ptr : m.get_child_meshes()) {
                if (child_ptr->top_cell_dimension() != 1)
                    continue; // only deal with edge child meshes
                auto& child_mmmanager = child_ptr->m_multi_mesh_manager;
                int64_t child_id = child_mmmanager.child_id();
                auto child_hash_accessor = child_ptr->get_const_cell_hash_accessor();
                auto child_to_parent_handle = child_mmmanager.map_to_parent_handle;
                auto parent_to_child_handle = parent_mmmanager.children().at(child_id).map_handle;
                auto child_to_parent_accessor = child_ptr->create_accessor(child_to_parent_handle);
                auto parent_to_child_accessor = m.create_accessor(parent_to_child_handle);

                auto parent_to_child_data = Mesh::get_index_access(parent_to_child_accessor)
                                                .const_vector_attribute(ear_eid);
                Tuple parent_tuple =
                    wmtk::multimesh::utils::vector5_to_tuple(parent_to_child_data.head<5>());
                Tuple child_tuple =
                    wmtk::multimesh::utils::vector5_to_tuple(parent_to_child_data.tail<5>());
                parent_tuple = m.resurrect_tuple(parent_tuple, parent_hash_accessor);
                child_tuple = child_ptr->resurrect_tuple(child_tuple, child_hash_accessor);

                // TODO: Currently using global ids to identify the tuple
                Tuple opt_tuple =
                    m.tuple_from_global_ids(ear_fid_other, ears[1].eid, fmoe.m_spine_vids[1]);

                if (m.id_vertex(parent_tuple) == ear_vid) {
                    // opt_tuple = m.switch_vertex(opt_tuple);
                    wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                        parent_to_child_accessor,
                        child_to_parent_accessor,
                        opt_tuple,
                        child_tuple);
                } else {
                    opt_tuple = m.switch_vertex(opt_tuple);
                    wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                        parent_to_child_accessor,
                        child_to_parent_accessor,
                        opt_tuple,
                        child_tuple);
                }
            }
        }
    }
}


// edge -> edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    EdgeMesh&,
    const simplex::Simplex&,
    const edge_mesh::EdgeOperationData& parent_tmoe,
    EdgeMesh&,
    const simplex::Simplex&,
    const edge_mesh::EdgeOperationData&) const
{
    throw std::runtime_error("not implemented");
}

// tri -> edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TriMesh& parent_mesh,
    const simplex::Simplex&,
    const tri_mesh::EdgeOperationData& parent_tmoe,
    EdgeMesh& child_mesh,
    const simplex::Simplex&,
    const edge_mesh::EdgeOperationData& child_emoe) const
{
    const auto& parent_incident_datas = parent_tmoe.incident_face_datas();
    const auto& parent_spine_v = parent_tmoe.incident_vids();

    auto& parent_mmmanager = parent_mesh.m_multi_mesh_manager;
    auto& child_mmmanager = child_mesh.m_multi_mesh_manager;
    auto child_to_parent_handle = child_mmmanager.map_to_parent_handle;
    int64_t child_id = child_mmmanager.child_id();
    auto parent_to_child_handle = parent_mmmanager.children().at(child_id).map_handle;
    auto child_to_parent_accessor = child_mesh.create_accessor(child_to_parent_handle);
    auto parent_to_child_accessor = parent_mesh.create_accessor(parent_to_child_handle);


    // update the new edges added by split
    for (int64_t index = 0; index < 2; ++index) {
        // we can choose f_parent on either side, here we choose 0
        int64_t f_parent = parent_incident_datas[0].split_f[index];

        const int64_t e_child = child_emoe.m_split_e[index];
        const int64_t e_parent = parent_tmoe.split_spine_eids[index];

        if (f_parent == -1 || e_child == -1 || e_parent == -1) {
            continue;
        }

        const int64_t v_child = child_emoe.m_spine_vids[index];
        const int64_t v_parent = parent_spine_v[index];

        const Tuple parent_tuple = parent_mesh.tuple_from_global_ids(f_parent, e_parent, v_parent);
        const Tuple child_tuple = child_mesh.tuple_from_global_ids(e_child, v_child);

        assert(parent_mesh.is_valid_slow(parent_tuple));
        assert(child_mesh.is_valid_slow(child_tuple));

        wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
            parent_to_child_accessor,
            child_to_parent_accessor,
            parent_tuple,
            child_tuple);
    }
}
// tri -> tri
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TriMesh& parent_mesh,
    const simplex::Simplex&,
    const tri_mesh::EdgeOperationData& parent_tmoe,
    TriMesh& child_mesh,
    const simplex::Simplex&,
    const tri_mesh::EdgeOperationData& child_tmoe) const
{
    // logger().trace(
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

    int64_t child_id = child_mmmanager.child_id();
    auto parent_to_child_handle = parent_mmmanager.children().at(child_id).map_handle;
    auto child_to_parent_accessor = child_mesh.create_accessor(child_to_parent_handle);
    auto parent_to_child_accessor = parent_mesh.create_accessor(parent_to_child_handle);

    // logger().trace(
    //     "Child had {} datas, parent had {} datas",
    //     child_incident_datas.size(),
    //     child_incident_datas.size());

    for (const auto& child_data : child_incident_datas) {
        int64_t target_parent_fid = parent_global_cid(child_to_parent_accessor, child_data.fid);
        //    logger().trace(
        //        "[{}=>{}] child data started with gid {}->{} and parent had {}",
        //        fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
        //        fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
        //        child_data.fid,
        //        fmt::join(child_data.split_f, ","),
        //        target_parent_fid);

        for (const auto& parent_data : parent_incident_datas) {
            // logger().trace(
            //     "Check for parent fid {} to be {} ({})",
            //     parent_data.fid,
            //     target_parent_fid,
            //     parent_data.fid == target_parent_fid);
            if (parent_data.fid == target_parent_fid) {
                // if there was a parent mapping we definitely need to update the edges
                const auto& child_split_f = child_data.split_f;
                const auto& parent_split_f = parent_data.split_f;


                for (int64_t index = 0; index < 2; ++index) {
                    int64_t f_child = child_split_f[index];
                    int64_t f_parent = parent_split_f[index];
                    if (f_child == -1 || f_parent == -1) {
                        continue;
                    }

                    int64_t e_child = child_data.ears[index].eid;
                    int64_t e_parent = parent_data.ears[index].eid;

                    int64_t v_child = child_spine_v[index];
                    int64_t v_parent = parent_spine_v[index];

                    const Tuple parent_tuple =
                        parent_mesh.tuple_from_global_ids(f_parent, e_parent, v_parent);
                    const Tuple child_tuple =
                        child_mesh.tuple_from_global_ids(f_child, e_child, v_child);
                    // logger().trace(
                    //     "[{}=>{}] combining these setes of GIDS: Parent: {} {} {} {}; Child: {}
                    //     {} "
                    //     "{} {}",
                    //   fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
                    //   fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
                    //   f_parent,
                    //   e_parent,
                    //   v_parent,

                    //  wmtk::utils::TupleInspector::as_string(parent_tuple),
                    //  f_child,
                    //  e_child,
                    //  v_child,
                    //  wmtk::utils::TupleInspector::as_string(child_tuple));


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
    // logger().trace(
    //    "[{0}=>{1}] updating hashes for {0}",
    //    fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
    //    fmt::join(child_mesh.absolute_multi_mesh_id(), ","));
    // logger().trace(
    //    "[{0}=>{1}] updating hashes for {1}",
    //    fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
    //    fmt::join(child_mesh.absolute_multi_mesh_id(), ","));

    // update_all_hashes(child_mesh,
    // child_tmoe.global_simplex_ids_with_potentially_modified_hashes);
}

// tet -> edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh&,
    const simplex::Simplex&,
    const tet_mesh::EdgeOperationData&,
    EdgeMesh&,
    const simplex::Simplex&,
    const edge_mesh::EdgeOperationData&) const
{
    throw std::runtime_error("not implemented");
}
// tet -> tri
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh&,
    const simplex::Simplex&,
    const tet_mesh::EdgeOperationData&,
    TriMesh&,
    const simplex::Simplex&,
    const tri_mesh::EdgeOperationData&) const
{
    throw std::runtime_error("not implemented");
}
// tet -> tet
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh& parent_mesh,
    const simplex::Simplex&,
    const tet_mesh::EdgeOperationData& parent_tmoe,
    TetMesh& child_mesh,
    const simplex::Simplex&,
    const tet_mesh::EdgeOperationData& child_tmoe) const
{
    const auto& parent_incident_tet_datas = parent_tmoe.incident_tet_datas();
    const auto& parent_incident_tet_datas = child_tmoe.incident_tet_datas();

    const auto& parent_incident_face_datas = parent_tmoe.incident_face_datas();
    const auto& child_incident_face_datas = child_tmoe.incident_face_datas();

    const auto& parent_spine_v = parent_tmoe.incident_vids();
    const auto& child_spine_v = child_tmoe.incident_vids();

    auto& parent_mmmanager = parent_mesh.m_multi_mesh_manager;
    auto& child_mmmanager = child_mesh.m_multi_mesh_manager;

    auto child_to_parent_handle = child_mmmanager.map_to_parent_handle;

    int64_t child_id = child_mmmanager.child_id();
    auto parent_to_child_handle = parent_mmmanager.children().at(child_id).map_handle;
    auto child_to_parent_accessor = child_mesh.create_accessor(child_to_parent_handle);
    auto parent_to_child_accessor = parent_mesh.create_accessor(parent_to_child_handle);

    for (const auto& child_data : parent_incident_tet_datas) {
        int64_t target_parent_tid = parent_global_cid(child_to_parent_accessor, child_data.tid);

        for (const auto& parent_data : parent_incident_tet_datas) {
            if (parent_data.tid == target_parent_tid) {
                const auto& child_split_t = child_data.split_t;
                const auto& parent_split_t = parent_data.split_t;

                for (int64_t index = 0; index < 2; ++index) {
                    int64_t t_child = child_split_t[index];
                    int64_t t_parent = parent_split_t[index];

                    if (t_child == -1 || t_parent == -1) {
                        continue; // TODO: why need this check?
                    }

                    int64_t f_child = child_data.ears[index].fid;
                    int64_t f_parent = parent_data.ears[index].fid;

                    int64_t e_child =
                        child_incident_face_datas[child_data.incident_face_data_idx[1]]
                            .ear_eids[index];
                    int64_t e_parent =
                        parent_incident_face_datas[parent_data.incident_face_data_idx[1]]
                            .ear_eids[index];

                    int64_t v_child = child_spine_v[index];
                    int64_t v_parent = parent_spine_v[index];

                    // TODO: why is this tuple selected? why not others?
                    // TODO: implement tuple_from_global_ids for tetmesh
                    const Tuple parent_tuple =
                        parent_mesh.tuple_from_global_ids(t_parent, f_parent, e_parent, v_parent);
                    const Tuple child_tuple =
                        child_mesh.tuple_from_global_ids(t_child, f_child, e_child, v_child);

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
}


// edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    EdgeMesh& parent_mesh,
    const simplex::Simplex&,
    const edge_mesh::EdgeOperationData& parent_emoe) const
{
    // if there's a child mesh then lets disallow this
#if !defined(NDEBUG)
    if (parent_mesh.get_child_meshes().size() > 0) {
        throw std::runtime_error("not implemented");
    }
#endif
    logger().error("EdgeMesh update Not implemented!");
}

// tri
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TriMesh& parent_mesh,
    const simplex::Simplex&,
    const tri_mesh::EdgeOperationData& parent_fmoe)
{
    std::vector<std::tuple<int64_t, std::array<int64_t, 2>>> parent_split_cell_maps;
    const auto& parent_incident_datas = parent_fmoe.incident_face_datas();
    for (const auto& parent_data : parent_incident_datas) {
        if (parent_data.split_f[0] == -1) break;
        parent_split_cell_maps.emplace_back(parent_data.fid, parent_data.split_f);
    }
    // TODO: update the ear edges here?

    if (parent_fmoe.is_collapse) {
        update_ear_replacement(parent_mesh, parent_fmoe);
    }
    update_all_hashes(
        parent_mesh,
        parent_fmoe.global_simplex_ids_with_potentially_modified_hashes,
        parent_split_cell_maps);
}

// tet
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh& parent_mesh,
    const simplex::Simplex&,
    const tet_mesh::EdgeOperationData& parent_tmoe)
{
    // if there's a child mesh then lets disallow this
    if (parent_mesh.get_child_meshes().size() > 0) {
        throw std::runtime_error("not implemented");
    }
}

int64_t UpdateEdgeOperationMultiMeshMapFunctor::child_global_cid(
    const attribute::ConstAccessor<int64_t>& parent_to_child,
    int64_t parent_gid) const
{
    return MultiMeshManager::child_global_cid(parent_to_child, parent_gid);
}
int64_t UpdateEdgeOperationMultiMeshMapFunctor::parent_global_cid(
    const attribute::ConstAccessor<int64_t>& child_to_parent,
    int64_t child_gid) const
{
    return MultiMeshManager::parent_global_cid(child_to_parent, child_gid);
}

} // namespace wmtk::operations::utils
