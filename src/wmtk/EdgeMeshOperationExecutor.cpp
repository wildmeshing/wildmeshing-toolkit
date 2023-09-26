
#include "EdgeMeshOperationExecutor.hpp"

namespace wmtk {
// constructor
EdgeMesh::EdgeMeshOperationExecutor::EdgeMeshOperationExecutor(
    EdgeMesh& m,
    const Tuple& operating_tuple,
    Accessor<long>& hash_acc)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge)}}
    , ee_accessor(m.create_accessor<long>(m.m_ee_handle))
    , ev_accessor(m.create_accessor<long>(m.m_ev_handle))
    , ve_accessor(m.create_accessor<long>(m.m_ve_handle))
    , hash_accessor(hash_acc)
    , m_mesh(m)
    , m_operating_tuple(operating_tuple)

{
    // store ids of edge and incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(m_mesh.switch_vertex(m_operating_tuple));

    const SimplicialComplex edge_closed_star =
        SimplicialComplex::closed_star(m_mesh, Simplex::edge(operating_tuple));

    // get all faces incident to the edge
    for (const Simplex& f : edge_closed_star.get_faces()) {
        // m_incident_face_datas.emplace_back(get_incident_face_data(f.tuple()));
    }

    // update hash on all faces in the two-ring neighborhood
    SimplicialComplex hash_update_region(m);
    for (const Simplex& v : edge_closed_star.get_vertices()) {
        const SimplicialComplex v_closed_star = SimplicialComplex::closed_star(m_mesh, v);
        hash_update_region.unify_with_complex(v_closed_star);
    }
    for (const Simplex& f : hash_update_region.get_faces()) {
        cell_ids_to_update_hash.push_back(m_mesh.id(f));
    }
}

void EdgeMesh::EdgeMeshOperationExecutor::delete_simplices()
{
    for (size_t d = 0; d < simplex_ids_to_delete.size(); ++d) {
        for (const long id : simplex_ids_to_delete[d]) {
            flag_accessors[d].index_access().scalar_attribute(id) = 0;
        }
    }
}

void EdgeMesh::EdgeMeshOperationExecutor::update_cell_hash()
{
    m_mesh.update_cell_hashes(cell_ids_to_update_hash, hash_accessor);
}

const std::array<std::vector<long>, 3>
EdgeMesh::EdgeMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const EdgeMesh& m)
{
    const SimplicialComplex sc = SimplicialComplex::open_star(m, Simplex::edge(tuple));
    std::array<std::vector<long>, 3> ids;
    for (const Simplex& s : sc.get_simplices()) {
        ids[get_simplex_dimension(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

const std::array<std::vector<long>, 3>
EdgeMesh::EdgeMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const EdgeMesh& m)
{
    const SimplicialComplex vertex_open_star =
        SimplicialComplex::open_star(m, Simplex::vertex(tuple));
    const SimplicialComplex edge_closed_star =
        SimplicialComplex::closed_star(m, Simplex::edge(tuple));

    const SimplicialComplex sc =
        SimplicialComplex::get_intersection(vertex_open_star, edge_closed_star);

    std::array<std::vector<long>, 3> ids;
    for (const Simplex& s : sc.get_simplices()) {
        ids[get_simplex_dimension(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

std::vector<std::vector<Tuple>>
EdgeMesh::EdgeMeshOperationExecutor::prepare_operating_tuples_for_child_meshes() const
{
    std::vector<std::vector<Tuple>> vec_t_child(m_incident_face_datas.size());
    for (long i = 0; i < long(m_incident_face_datas.size()); ++i) {
        vec_t_child[i] = MultiMeshManager::map_edge_tuple_to_all_children(
            m_mesh,
            m_incident_face_datas[i].local_operating_tuple);
    }
    // clean up redundant
    if (m_incident_face_datas.size() > 1) {
        for (long j = 0; j < long(vec_t_child[1].size()); ++j) {
            auto child_mesh_ptr = m_mesh.multi_mesh_manager.child_meshes[j];
            if (vec_t_child[1][j].is_null() || vec_t_child[0][j].is_null()) {
                continue;
            }
            if (child_mesh_ptr->simplices_are_equal(
                    Simplex::edge(vec_t_child[0][j]),
                    Simplex::edge(vec_t_child[1][j]))) {
                vec_t_child[1][j] = Tuple(); // redundant, set as null tuple
            }
        }
    }
    return vec_t_child;
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::split_edge()
{
    if (!m_mesh.multi_mesh_manager.is_parent_mesh()) {
        return split_edge_single_mesh();
    } else {
        std::vector<std::vector<Tuple>> vec_t_child = prepare_operating_tuples_for_child_meshes();

        // do split on parent_mesh
        Tuple ret_tuple = split_edge_single_mesh();

        for (auto child_mesh_ptr : m_mesh.multi_mesh_manager.child_meshes) {
            long child_id = child_mesh_ptr->multi_mesh_manager.child_id();
            if (child_mesh_ptr->top_simplex_type() == PrimitiveType::Face) {
                // this child_mesh is a EdgeMesh
                EdgeMesh& child_tri_mesh = *std::static_pointer_cast<EdgeMesh>(child_mesh_ptr);

                std::vector<std::pair<long, long>> child_new_cell_ids;
                for (long i = 0; i < long(m_incident_face_datas.size()); ++i) {
                    Tuple t_child = vec_t_child[i][child_id];
                    if (t_child.is_null()) {
                        if (child_new_cell_ids.size() <= i) child_new_cell_ids.emplace_back(-1, -1);
                        continue;
                    }
                    auto child_hash_acc = child_tri_mesh.get_cell_hash_accessor();
                    EdgeMesh::EdgeMeshOperationExecutor executor_child(
                        child_tri_mesh,
                        t_child,
                        child_hash_acc);
                    executor_child.split_edge();
                    for (auto child_incident_face_data : executor_child.m_incident_face_datas) {
                        child_new_cell_ids.emplace_back(
                            child_incident_face_data.split_f0,
                            child_incident_face_data.split_f1);
                    }
                }

                assert(child_new_cell_ids.size() == m_incident_face_datas.size());

                // update_hash on new cells
                for (long i = 0; i < long(m_incident_face_datas.size()); i++) {
                    const auto& split_fs_child = child_new_cell_ids[i];
                    long split_f0_child = split_fs_child.first;
                    long split_f1_child = split_fs_child.second;

                    const auto& incident_face_data = m_incident_face_datas[i];
                    long split_f0_parent = incident_face_data.split_f0;
                    long split_f1_parent = incident_face_data.split_f1;

                    Tuple tuple_child = (split_f0_child == -1)
                                            ? Tuple()
                                            : child_tri_mesh.face_tuple_from_id(split_f0_child);
                    Tuple tuple_parent = m_mesh.face_tuple_from_id(split_f0_parent);

                    if (!tuple_child.is_null()) {
                        MultiMeshManager::write_tuple_map_attribute(
                            child_tri_mesh.multi_mesh_manager.map_to_parent_handle,
                            child_tri_mesh,
                            tuple_child,
                            tuple_parent);
                    }
                    MultiMeshManager::write_tuple_map_attribute(
                        m_mesh.multi_mesh_manager.map_to_child_handles[child_id],
                        m_mesh,
                        tuple_parent,
                        tuple_child);

                    tuple_child = (split_f1_child == -1)
                                      ? Tuple()
                                      : child_tri_mesh.face_tuple_from_id(split_f1_child);
                    tuple_parent = m_mesh.face_tuple_from_id(split_f1_parent);
                    if (!tuple_child.is_null()) {
                        MultiMeshManager::write_tuple_map_attribute(
                            child_tri_mesh.multi_mesh_manager.map_to_parent_handle,
                            child_tri_mesh,
                            tuple_child,
                            tuple_parent);
                    }
                    MultiMeshManager::write_tuple_map_attribute(
                        m_mesh.multi_mesh_manager.map_to_child_handles[child_id],
                        m_mesh,
                        tuple_parent,
                        tuple_child);
                }

                // update_hash on neighboring cells
                update_hash_in_map(child_tri_mesh);
            }
        }
        return ret_tuple;
    }
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::split_edge_single_mesh()
{
    simplex_ids_to_delete = get_split_simplices_to_delete(m_operating_tuple, m_mesh);

    // create new vertex (center)
    std::vector<long> new_vids = this->request_simplex_indices(PrimitiveType::Vertex, 1);
    assert(new_vids.size() == 1);
    const long v_new = new_vids[0];

    // create new edges (spine)
    std::vector<long> new_eids = this->request_simplex_indices(PrimitiveType::Edge, 2);
    assert(new_eids.size() == 2);

    for (IncidentFaceData& face_data : m_incident_face_datas) {
        replace_incident_face(v_new, new_eids, face_data);
    }
    assert(m_incident_face_datas.size() <= 2);
    if (m_incident_face_datas.size() > 1) {
        connect_faces_across_spine();
    }

    update_cell_hash();
    delete_simplices();
    // return Tuple new_fid, new_vid that points
    const long new_tuple_fid = m_incident_face_datas[0].split_f1;
    Tuple ret = m_mesh.edge_tuple_from_id(new_eids[1]);
    if (m_mesh.id_vertex(ret) != v_new) {
        ret = m_mesh.switch_vertex(ret);
    }
    if (m_mesh.id_face(ret) != new_tuple_fid) {
        ret = m_mesh.switch_face(ret);
    }
    assert(m_mesh.is_valid_slow(ret));

    return ret;
    // return m_mesh.with_different_cid(m_operating_tuple, m_incident_face_datas[0].split_f0);
}

void EdgeMesh::EdgeMeshOperationExecutor::update_hash_in_map(EdgeMesh& child_mesh)
{
    long child_id = child_mesh.multi_mesh_manager.child_id();
    auto child_hash_accessor = child_mesh.get_cell_hash_accessor();

    for (auto parent_cell_id : cell_ids_to_update_hash) {
        auto [t_parent_old, t_child_old] = MultiMeshManager::read_tuple_map_attribute(
            m_mesh.multi_mesh_manager.map_to_child_handles[child_id],
            m_mesh,
            m_mesh.tuple_from_id(m_mesh.top_simplex_type(), parent_cell_id));

        long parent_cell_hash = m_mesh.get_cell_hash_slow(parent_cell_id);
        Tuple t_parent_new = t_parent_old.with_updated_hash(parent_cell_hash);

        if (t_child_old.is_null()) {
            MultiMeshManager::write_tuple_map_attribute(
                m_mesh.multi_mesh_manager.map_to_child_handles[child_id],
                m_mesh,
                t_parent_new,
                Tuple());
        } else {
            long child_cell_hash =
                child_hash_accessor.index_access().const_scalar_attribute(t_child_old.m_global_cid);
            Tuple t_child_new = t_child_old.with_updated_hash(child_cell_hash);
            MultiMeshManager::write_tuple_map_attribute(
                m_mesh.multi_mesh_manager.map_to_child_handles[child_id],
                m_mesh,
                t_parent_new,
                t_child_new);

            MultiMeshManager::write_tuple_map_attribute(
                child_mesh.multi_mesh_manager.map_to_parent_handle,
                child_mesh,
                t_child_new,
                t_parent_new);
        }
    }
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::collapse_edge()
{
    if (!m_mesh.multi_mesh_manager.is_parent_mesh()) {
        return collapse_edge_single_mesh();
    } else {
        std::vector<std::vector<Tuple>> vec_t_child = prepare_operating_tuples_for_child_meshes();

        // do collapse on parent_mesh
        Tuple ret_tuple = collapse_edge_single_mesh();

        for (auto child_mesh_ptr : m_mesh.multi_mesh_manager.child_meshes) {
            long child_id = child_mesh_ptr->multi_mesh_manager.child_id();

            if (child_mesh_ptr->top_simplex_type() == PrimitiveType::Face) {
                // this child_mesh is a EdgeMesh
                EdgeMesh& child_tri_mesh = *std::static_pointer_cast<EdgeMesh>(child_mesh_ptr);

                for (long i = 0; i < long(m_incident_face_datas.size()); ++i) {
                    Tuple t_child = vec_t_child[i][child_id];
                    if (t_child.is_null()) {
                        continue;
                    }
                    auto child_hash_acc = child_tri_mesh.get_cell_hash_accessor();
                    EdgeMesh::EdgeMeshOperationExecutor executor_child(
                        child_tri_mesh,
                        t_child,
                        child_hash_acc);
                    executor_child.collapse_edge();
                }
                // update_hash
                update_hash_in_map(child_tri_mesh);
            }
        }

        return ret_tuple;
    }
}


Tuple EdgeMesh::EdgeMeshOperationExecutor::collapse_edge_single_mesh()
{
    simplex_ids_to_delete = get_collapse_simplices_to_delete(m_operating_tuple, m_mesh);

    // must collect star before changing connectivity
    const SimplicialComplex v0_star =
        SimplicialComplex::closed_star(m_mesh, Simplex::vertex(m_operating_tuple));


    connect_ears();

    const long& v0 = m_spine_vids[0];
    const long& v1 = m_spine_vids[1];

    // replace v0 by v1 in incident faces
    for (const Simplex& f : v0_star.get_faces()) {
        const long fid = m_mesh.id(f);
        auto fv = fv_accessor.index_access().vector_attribute(fid);
        for (long i = 0; i < 3; ++i) {
            if (fv[i] == v0) {
                fv[i] = v1;
                break;
            }
        }
    }

    const long& ret_eid = m_incident_face_datas[0].ears[1].eid;
    const long& ret_vid = m_spine_vids[1];
    const long& ef0 = m_incident_face_datas[0].ears[0].fid;
    const long& ef1 = m_incident_face_datas[0].ears[1].fid;

    const long new_tuple_fid = (ef0 > -1) ? ef0 : ef1;

    update_cell_hash();
    delete_simplices();

    Tuple ret = m_mesh.edge_tuple_from_id(ret_eid);
    if (m_mesh.id_vertex(ret) != ret_vid) {
        ret = m_mesh.switch_vertex(ret);
    }
    assert(m_mesh.id_vertex(ret) == ret_vid);
    if (m_mesh.id_face(ret) != new_tuple_fid) {
        ret = m_mesh.switch_face(ret);
    }
    assert(m_mesh.id_face(ret) == new_tuple_fid);
    assert(m_mesh.is_valid_slow(ret));


    return ret;

    // return a ccw tuple from left ear if it exists, otherwise return a ccw tuple from right ear
    // return m_mesh.tuple_from_id(PrimitiveType::Vertex, v1);
}

std::vector<long> EdgeMesh::EdgeMeshOperationExecutor::request_simplex_indices(
    const PrimitiveType type,
    long count)
{
    m_mesh.reserve_attributes(type, m_mesh.capacity(type) + count);

    return m_mesh.request_simplex_indices(type, count);
}

} // namespace wmtk
