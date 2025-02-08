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
constexpr static PrimitiveType PF = PrimitiveType::Triangle;
constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;
} // namespace
void UpdateEdgeOperationMultiMeshMapFunctor::update_all_hashes(
    Mesh& m,
    const std::vector<std::vector<std::tuple<int64_t, std::vector<Tuple>>>>& simplices_to_update,
    const std::vector<std::tuple<int64_t, std::array<int64_t, 2>>>& split_cell_maps) const
{
    // assert(m.top_cell_dimension() + 1 == simplices_to_update.size());
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
    const auto& parent_incident_vids = fmoe.incident_vids();

    for (const auto& parent_data : parent_incident_datas) {
        // std::cout << parent_data.fid << " has been processed" << std::endl;

        for (auto child_ptr : m.get_child_meshes()) {
            // no ear replcaement required for free child meshes
            if (child_ptr->is_free()) {
                continue;
            }
            if (child_ptr->top_cell_dimension() != 1) {
                continue; // only deal with child edgemeshes
            }

            const auto& child_mmmanager = child_ptr->m_multi_mesh_manager;
            int64_t child_id = child_mmmanager.child_id();
            auto child_to_parent_handle = child_mmmanager.map_to_parent_handle;
            auto parent_to_child_handle = parent_mmmanager.children().at(child_id).map_handle;
            auto child_to_parent_accessor = child_ptr->create_accessor(child_to_parent_handle);
            auto parent_to_child_accessor = m.create_accessor(parent_to_child_handle);
            auto child_cell_flag_accessor = child_ptr->get_const_flag_accessor(PrimitiveType::Edge);

            std::vector<std::pair<Tuple, Tuple>> update_pairs;

            for (int ear_index = 0; ear_index < 2; ++ear_index) {
                const int64_t parent_ear_eid_old = parent_data.ears[ear_index].eid;
                const int64_t parent_merged_eid = parent_data.new_edge_id;
                const int64_t parent_new_fid = parent_data.merged_edge_fid;


                assert(parent_merged_eid != -1);
                assert(parent_new_fid != -1);

                auto parent_to_child_data = Mesh::get_index_access(parent_to_child_accessor)
                                                .const_vector_attribute(parent_ear_eid_old);

                Tuple parent_tuple, child_tuple;
                std::tie(parent_tuple, child_tuple) =
                    wmtk::multimesh::utils::vectors_to_tuples(parent_to_child_data);

                if (child_tuple.is_null()) {
                    // not child_tuple on this parent edge
                    continue;
                }


                //  check also the flag accessor of child mesh
                const bool child_tuple_exists = child_cell_flag_accessor.is_active(child_tuple);
                if (!child_tuple_exists) {
                    continue;
                }

                const int64_t parent_old_vid =
                    m.parent_scope([&]() { return m.id_vertex(parent_tuple); });


                int64_t parent_new_vid = -1;

                if (parent_ear_eid_old != parent_merged_eid) {
                    // other side
                    if (parent_old_vid == parent_incident_vids[0]) {
                        parent_new_vid = parent_incident_vids[1];
                    } else {
                        parent_new_vid = parent_old_vid;
                    }
                } else {
                    // same side
                    parent_new_vid = parent_old_vid;
                }

                assert(parent_new_vid != -1);

                Tuple new_parent_tuple =
                    m.tuple_from_global_ids(parent_new_fid, parent_merged_eid, parent_new_vid);

                update_pairs.push_back(std::make_pair(new_parent_tuple, child_tuple));
            }

            for (const auto& pair : update_pairs) {
                wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                    parent_to_child_accessor,
                    child_to_parent_accessor,
                    pair.first,
                    pair.second);
            }
        }
    }
}

void UpdateEdgeOperationMultiMeshMapFunctor::update_ear_replacement(
    TetMesh& m,
    const tet_mesh::EdgeOperationData& tmoe) const
{
    const auto& parent_incident_tet_datas = tmoe.incident_tet_datas();
    const auto& parent_incident_face_datas = tmoe.incident_face_datas();
    auto parent_mmmanager = m.m_multi_mesh_manager;

    for (const auto& parent_data : parent_incident_tet_datas) {
        for (auto child_ptr : m.get_child_meshes()) {
            // no ear replcaement required for free child meshes
            if (child_ptr->is_free()) {
                continue;
            }
            if (child_ptr->top_cell_dimension() == 2) {
                // handle with child tri mesh
                // update merge faces here
                const auto& child_mmmanager = child_ptr->m_multi_mesh_manager;
                const int64_t child_id = child_mmmanager.child_id();
                const auto child_to_parent_handle = child_mmmanager.map_to_parent_handle;
                const auto parent_to_child_handle =
                    parent_mmmanager.children().at(child_id).map_handle;
                auto child_to_parent_accessor = child_ptr->create_accessor(child_to_parent_handle);
                auto parent_to_child_accessor = m.create_accessor(parent_to_child_handle);
                auto child_cell_flag_accessor =
                    child_ptr->get_const_flag_accessor(PrimitiveType::Triangle);

                std::vector<std::pair<Tuple, Tuple>> update_pairs;

                for (int ear_index = 0; ear_index < 2; ++ear_index) {
                    const int64_t parent_ear_fid_old = parent_data.ears[ear_index].fid;
                    const int64_t parent_merged_fid = parent_data.new_face_id;
                    const int64_t parent_new_tid =
                        parent_data.merged_face_tid; // can be move to outer loop

                    assert(parent_merged_fid != -1);
                    assert(parent_new_tid != -1);

                    auto parent_to_child_data = Mesh::get_index_access(parent_to_child_accessor)
                                                    .const_vector_attribute(parent_ear_fid_old);

                    // TUPLE_SIZE is the number of tuples in terms of lon
                    Tuple parent_tuple, child_tuple;
                    std::tie(parent_tuple, child_tuple) =
                        wmtk::multimesh::utils::vectors_to_tuples(parent_to_child_data);

                    if (child_tuple.is_null()) {
                        // not child_tuple on this parent face
                        continue;
                    }


                    bool child_tuple_exists = child_cell_flag_accessor.is_active(child_tuple);
                    if (!child_tuple_exists) {
                        continue;
                    }

                    const int64_t parent_old_eid =
                        m.parent_scope([&]() { return m.id_edge(parent_tuple); });
                    const int64_t parent_old_vid =
                        m.parent_scope([&]() { return m.id_vertex(parent_tuple); });


                    // get the corresponding new eid and vid of parent
                    int64_t parent_new_eid = -1;
                    int64_t parent_new_vid = -1;

                    if (parent_ear_fid_old != parent_merged_fid) {
                        // other side
                        if (parent_old_eid == parent_data.e02) {
                            parent_new_eid = parent_data.e12;
                        } else if (parent_old_eid == parent_data.e03) {
                            parent_new_eid = parent_data.e13;
                        } else if (parent_old_eid == parent_data.e23) {
                            parent_new_eid = parent_data.e23;
                        }

                        if (parent_old_vid == parent_data.v0) {
                            parent_new_vid = parent_data.v1;
                        } else if (parent_old_vid == parent_data.v2) {
                            parent_new_vid = parent_data.v2;
                        } else if (parent_old_vid == parent_data.v3) {
                            parent_new_vid = parent_data.v3;
                        }
                    } else {
                        // same side
                        parent_new_eid = parent_old_eid;
                        parent_new_vid = parent_old_vid;
                    }

                    assert(parent_new_eid != -1);
                    assert(parent_new_vid != -1);

                    Tuple new_parent_tuple = m.tuple_from_global_ids(
                        parent_new_tid,
                        parent_merged_fid,
                        parent_new_eid,
                        parent_new_vid);

                    update_pairs.push_back(std::make_pair(new_parent_tuple, child_tuple));
                }

                for (const auto& pair : update_pairs) {
                    wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                        parent_to_child_accessor,
                        child_to_parent_accessor,
                        pair.first,
                        pair.second);
                }

            } else if (child_ptr->top_cell_dimension() == 1) {
                // handle with child edge mesh
                // update merge edges here
                // there are three ear edges per side
                const auto& child_mmmanager = child_ptr->m_multi_mesh_manager;
                int64_t child_id = child_mmmanager.child_id();
                auto child_to_parent_handle = child_mmmanager.map_to_parent_handle;
                auto parent_to_child_handle = parent_mmmanager.children().at(child_id).map_handle;
                auto child_to_parent_accessor = child_ptr->create_accessor(child_to_parent_handle);
                auto parent_to_child_accessor = m.create_accessor(parent_to_child_handle);
                auto child_cell_flag_accessor =
                    child_ptr->get_const_flag_accessor(PrimitiveType::Edge);

                std::vector<std::pair<Tuple, Tuple>> update_pairs;

                for (int ear_index = 0; ear_index < 2; ++ear_index) {
                    const int64_t parent_ear_fid_old = parent_data.ears[ear_index].fid;
                    const int64_t parent_merged_fid = parent_data.new_face_id;
                    const int64_t parent_new_tid = parent_data.merged_face_tid;


                    std::array<int64_t, 3> parent_old_eids;
                    std::array<int64_t, 3> parent_new_eids = {
                        {parent_data.e12, parent_data.e13, parent_data.e23}};

                    if (parent_ear_fid_old != parent_merged_fid) {
                        parent_old_eids = {{parent_data.e02, parent_data.e03, parent_data.e23}};
                    } else {
                        parent_old_eids = {{parent_data.e12, parent_data.e13, parent_data.e23}};
                    }


                    assert(parent_merged_fid != -1);
                    assert(parent_new_tid != -1);

                    for (int i = 0; i < 3; ++i) {
                        auto parent_to_child_data = Mesh::get_index_access(parent_to_child_accessor)
                                                        .const_vector_attribute(parent_old_eids[i]);


                        Tuple parent_tuple, child_tuple;
                        std::tie(parent_tuple, child_tuple) =
                            wmtk::multimesh::utils::vectors_to_tuples(parent_to_child_data);

                        if (child_tuple.is_null()) {
                            // not child_tuple on this parent edge
                            continue;
                        }


                        bool child_tuple_exists = child_cell_flag_accessor.is_active(child_tuple);
                        if (!child_tuple_exists) {
                            continue;
                        }

                        const int64_t parent_old_vid =
                            m.parent_scope([&]() { return m.id_vertex(parent_tuple); });

                        int64_t parent_new_vid = -1;
                        if (parent_ear_fid_old != parent_merged_fid) {
                            // other side
                            if (parent_old_vid == parent_data.v0) {
                                parent_new_vid = parent_data.v1;
                            } else if (parent_old_vid == parent_data.v2) {
                                parent_new_vid = parent_data.v2;
                            } else if (parent_old_vid == parent_data.v3) {
                                parent_new_vid = parent_data.v3;
                            }
                        } else {
                            // same side
                            parent_new_vid = parent_old_vid;
                        }

                        assert(parent_new_vid != -1);

                        Tuple new_parent_tuple = m.tuple_from_global_ids(
                            parent_new_tid,
                            parent_merged_fid,
                            parent_new_eids[i],
                            parent_new_vid);

                        update_pairs.push_back(std::make_pair(new_parent_tuple, child_tuple));
                    }

                    for (const auto& pair : update_pairs) {
                        wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                            parent_to_child_accessor,
                            child_to_parent_accessor,
                            pair.first,
                            pair.second);
                    }
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

        assert(parent_mesh.is_valid(parent_tuple));
        assert(child_mesh.is_valid(child_tuple));

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
                    //     "[{}=>{}] combining these setes of GIDS: Parent: {} {} {} {}; Child:
                    //     {}
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


                    assert(parent_mesh.is_valid(parent_tuple));
                    assert(child_mesh.is_valid(child_tuple));

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
    // constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    // constexpr static PrimitiveType PE = PrimitiveType::Edge;
    // constexpr static PrimitiveType PF = PrimitiveType::Triangle;

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
    // child_tmoe.global_ids_to_potential_tuples);
}

// tet -> edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh& parent_mesh,
    const simplex::Simplex&,
    const tet_mesh::EdgeOperationData& parent_tmoe,
    EdgeMesh& child_mesh,
    const simplex::Simplex&,
    const edge_mesh::EdgeOperationData& child_emoe) const
{
    const auto& parent_incident_tet_datas = parent_tmoe.incident_tet_datas();
    const auto& parent_incident_face_datas = parent_tmoe.incident_face_datas();

    auto& parent_mmmanager = parent_mesh.m_multi_mesh_manager;
    auto& child_mmmanager = child_mesh.m_multi_mesh_manager;

    auto child_to_parent_handle = child_mmmanager.map_to_parent_handle;

    int64_t child_id = child_mmmanager.child_id();
    auto parent_to_child_handle = parent_mmmanager.children().at(child_id).map_handle;
    auto child_to_parent_accessor = child_mesh.create_accessor(child_to_parent_handle);
    auto parent_to_child_accessor = parent_mesh.create_accessor(parent_to_child_handle);

    int64_t target_parent_tid =
        parent_global_cid(child_to_parent_accessor, child_emoe.m_operating_edge_id);
    int64_t target_parent_local_fid =
        parent_local_fid(child_to_parent_accessor, child_emoe.m_operating_edge_id);
    for (const auto& parent_data : parent_incident_tet_datas) {
        if (parent_data.tid != target_parent_tid) continue;

        int64_t face_index = -1; // shoule be 0 or 1 after if
        for (int i = 0; i < 2; ++i) {
            if (parent_data.incident_face_local_fid[i] == target_parent_local_fid) {
                // target_parent_global_fid =
                //     parent_incident_face_datas[parent_data.incident_face_data_idx[i]].fid;
                face_index = i;
            }
        }
        assert(face_index != -1);

        for (int index = 0; index < 2; ++index) {
            const int64_t t_parent = parent_data.split_t[index];
            const int64_t f_parent =
                parent_incident_face_datas[parent_data.incident_face_data_idx[face_index]]
                    .split_f[index];

            const int64_t e_parent = parent_tmoe.new_spine_eids()[index];
            const int64_t e_child = child_emoe.m_split_e[index];

            if (t_parent == -1 || f_parent == -1 || e_child == -1 || e_parent == -1) {
                continue; // why?
            }

            const int64_t v_parent = parent_tmoe.incident_vids()[index];
            const int64_t v_child =
                child_emoe.m_spine_vids[index]; // these two tuples are in inversed direction, also
                                                // for tet->tri and tri->edge, shall we change?

            const Tuple parent_tuple =
                parent_mesh.tuple_from_global_ids(t_parent, f_parent, e_parent, v_parent);
            const Tuple child_tuple = child_mesh.tuple_from_global_ids(e_child, v_child);

            assert(parent_mesh.is_valid(parent_tuple));
            assert(child_mesh.is_valid(child_tuple));

            wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                parent_to_child_accessor,
                child_to_parent_accessor,
                parent_tuple,
                child_tuple);
        }
    }
}

// tet -> tri
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh& parent_mesh,
    const simplex::Simplex&,
    const tet_mesh::EdgeOperationData& parent_tmoe,
    TriMesh& child_mesh,
    const simplex::Simplex&,
    const tri_mesh::EdgeOperationData& child_fmoe) const
{
    const auto& parent_incident_tet_datas = parent_tmoe.incident_tet_datas();
    const auto& parent_incident_face_datas = parent_tmoe.incident_face_datas();

    const auto& child_incident_face_datas = child_fmoe.incident_face_datas();

    const auto& parent_spine_v = parent_tmoe.incident_vids();
    const auto& child_spine_v = child_fmoe.incident_vids();

    auto& parent_mmmanager = parent_mesh.m_multi_mesh_manager;
    auto& child_mmmanager = child_mesh.m_multi_mesh_manager;

    auto child_to_parent_handle = child_mmmanager.map_to_parent_handle;

    int64_t child_id = child_mmmanager.child_id();
    auto parent_to_child_handle = parent_mmmanager.children().at(child_id).map_handle;
    auto child_to_parent_accessor = child_mesh.create_accessor(child_to_parent_handle);
    auto parent_to_child_accessor = parent_mesh.create_accessor(parent_to_child_handle);

    for (const auto& child_data : child_incident_face_datas) {
        int64_t target_parent_tid = parent_global_cid(child_to_parent_accessor, child_data.fid);
        // get parent local fid or global fid?
        // try local here
        int64_t target_parent_local_fid =
            parent_local_fid(child_to_parent_accessor, child_data.fid);

        for (const auto& parent_data : parent_incident_tet_datas) {
            if (parent_data.tid != target_parent_tid) continue;
            int64_t face_index = -1; // shoule be 0 or 1 after if
            for (int i = 0; i < 2; ++i) {
                if (parent_data.incident_face_local_fid[i] == target_parent_local_fid) {
                    // target_parent_global_fid =
                    //     parent_incident_face_datas[parent_data.incident_face_data_idx[i]].fid;
                    face_index = i;
                }
            }
            assert(face_index != -1);

            for (int index = 0; index < 2; ++index) {
                // should check if parent child tuples are in the same orientation
                int64_t t_parent = parent_data.split_t[index];

                int64_t f_child = child_data.split_f[index];
                int64_t f_parent =
                    parent_incident_face_datas[parent_data.incident_face_data_idx[face_index]]
                        .split_f[index];

                if (t_parent == -1 || f_child == -1 || f_parent == -1) {
                    continue;
                }

                int64_t e_child = child_data.ears[index].eid;
                int64_t e_parent =
                    parent_incident_face_datas[parent_data.incident_face_data_idx[face_index]]
                        .ear_eids[index];

                int64_t v_child = child_spine_v[index];
                int64_t v_parent = parent_spine_v[index];

                const Tuple parent_tuple =
                    parent_mesh.tuple_from_global_ids(t_parent, f_parent, e_parent, v_parent);
                const Tuple child_tuple =
                    child_mesh.tuple_from_global_ids(f_child, e_child, v_child);

                assert(parent_mesh.is_valid(parent_tuple));
                assert(child_mesh.is_valid(child_tuple));

                wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                    parent_to_child_accessor,
                    child_to_parent_accessor,
                    parent_tuple,
                    child_tuple);
            }
        }
    }
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
    const auto& child_incident_tet_datas = child_tmoe.incident_tet_datas();

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

    for (const auto& child_data : child_incident_tet_datas) {
        int64_t target_parent_tid = parent_global_cid(child_to_parent_accessor, child_data.tid);

        for (const auto& parent_data : parent_incident_tet_datas) {
            if (parent_data.tid != target_parent_tid) continue;
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
                    child_incident_face_datas[child_data.incident_face_data_idx[1]].ear_eids[index];
                int64_t e_parent = parent_incident_face_datas[parent_data.incident_face_data_idx[1]]
                                       .ear_eids[index];

                int64_t v_child = child_spine_v[index];
                int64_t v_parent = parent_spine_v[index];

                // TODO: why is this tuple selected? why not others?
                const Tuple parent_tuple =
                    parent_mesh.tuple_from_global_ids(t_parent, f_parent, e_parent, v_parent);
                const Tuple child_tuple =
                    child_mesh.tuple_from_global_ids(t_child, f_child, e_child, v_child);

                assert(parent_mesh.is_valid(parent_tuple));
                assert(child_mesh.is_valid(child_tuple));

                wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                    parent_to_child_accessor,
                    child_to_parent_accessor,
                    parent_tuple,
                    child_tuple);
            }
        }
    }
}


// edge
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    EdgeMesh& parent_mesh,
    const simplex::Simplex&,
    const edge_mesh::EdgeOperationData& parent_emoe)
{
    return;
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
        parent_fmoe.global_ids_to_potential_tuples,
        parent_split_cell_maps);
}

// tet
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TetMesh& parent_mesh,
    const simplex::Simplex&,
    const tet_mesh::EdgeOperationData& parent_tmoe)
{
    std::vector<std::tuple<int64_t, std::array<int64_t, 2>>> parent_split_cell_maps;
    const auto& parent_incident_tet_datas = parent_tmoe.incident_tet_datas();
    for (const auto& parent_data : parent_incident_tet_datas) {
        if (parent_data.split_t[0] == -1) break; // no split datas, not a split function
        parent_split_cell_maps.emplace_back(parent_data.tid, parent_data.split_t);
    }

    if (parent_tmoe.is_collapse) {
        update_ear_replacement(parent_mesh, parent_tmoe);
    }
    update_all_hashes(
        parent_mesh,
        parent_tmoe.global_ids_to_potential_tuples,
        parent_split_cell_maps);
}

int64_t UpdateEdgeOperationMultiMeshMapFunctor::child_global_cid(
    const attribute::Accessor<int64_t, Mesh, Eigen::Dynamic>& parent_to_child,
    int64_t parent_gid) const
{
    return multimesh::MultiMeshManager::child_global_cid(parent_to_child, parent_gid);
}
int64_t UpdateEdgeOperationMultiMeshMapFunctor::parent_global_cid(
    const attribute::Accessor<int64_t, Mesh, Eigen::Dynamic>& child_to_parent,
    int64_t child_gid) const
{
    return multimesh::MultiMeshManager::parent_global_cid(child_to_parent, child_gid);
}

int64_t UpdateEdgeOperationMultiMeshMapFunctor::parent_local_fid(
    const attribute::Accessor<int64_t, Mesh, Eigen::Dynamic>& child_to_parent,
    int64_t child_gid) const
{
    return multimesh::MultiMeshManager::parent_local_fid(child_to_parent, child_gid);
}


/*
template <typename MeshType>
void UpdateEdgeOperationMultiMeshMapFunctor::update_maps(
    MeshType& mesh,
    const simplex::Simplex&,
    const EdgeOperationData&)
{
    // on split:
    // for each dimension
    //      for each boundary simplex
    //          update boundary facets using availalbe simplices (in split we have 2 choices,
    //          collapse we have to look at opposing ears each simplex that refers to an existing
    //          simplex must be replaced
    //
    // on collapse:
    // for each dimension
    //      for each boundary simplex
    //          if the face used an ear simplex
    //          update boundary facets using availalbe simplices (in split we have 2 choices,
    //          collapse we have to look at opposing ears each simplex that refers to an existing
    //          simplex must be replaced
    //
    //


    for (size_t j = 0; j < simplices_to_update.size(); ++j) {
        mesh.m_multi_mesh_manager
            .update_map_tuple_hashes(m, PTs[j], simplices_to_update[j], split_cell_maps);
    }
}
*/

} // namespace wmtk::operations::utils
