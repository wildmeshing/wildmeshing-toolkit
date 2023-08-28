
#include "TriMeshOperationExecutor.hpp"

namespace wmtk {

TriMesh::TriMeshOperationExecutor::IncidentFaceData
TriMesh::TriMeshOperationExecutor::get_incident_face_data(Tuple t)
{
    //         / \ 
    //  ear1  /   \  ear2
    //       /     \ 
    //      /       \ 
    //     X----t----

    // make sure that edge and vertex of the tuple are the same
    for (int i = 0; i < 3; ++i) {
        if (m_mesh.simplex_is_equal(Simplex::edge(t), Simplex::edge(m_operating_tuple))) {
            break;
        }
        t = m_mesh.next_edge(t);
    }
    assert(m_mesh.simplex_is_equal(Simplex::edge(t), Simplex::edge(m_operating_tuple)));

    if (!m_mesh.simplex_is_equal(Simplex::vertex(t), Simplex::vertex(m_operating_tuple))) {
        t = m_mesh.switch_vertex(t);
    }
    assert(m_mesh.simplex_is_equal(Simplex::vertex(t), Simplex::vertex(m_operating_tuple)));

    const Tuple ear1_edge = m_mesh.switch_edge(t);
    const Tuple ear2_edge = m_mesh.switch_edge(m_mesh.switch_vertex(t));

    IncidentFaceData face_data;
    face_data.fid = m_mesh.id_face(t);
    face_data.opposite_vid = m_mesh.id_vertex(m_mesh.switch_vertex(ear1_edge));


    // accessing ear face id through FF to make it work also at boundaries
    const long ear1_fid = ff_accessor.vector_attribute(ear1_edge)[ear1_edge.m_local_eid];
    const long ear2_fid = ff_accessor.vector_attribute(ear2_edge)[ear2_edge.m_local_eid];

    face_data.ears[0] = EarFace{
        /*.fid = */ ear1_fid,
        /*.eid = */ m_mesh.id_edge(ear1_edge)};

    face_data.ears[1] = EarFace{
        /*.fid = */ ear2_fid,
        /*.eid = */ m_mesh.id_edge(ear2_edge)};

    return face_data;
}

// constructor
TriMesh::TriMeshOperationExecutor::TriMeshOperationExecutor(
    TriMesh& m,
    const Tuple& operating_tuple)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge), m.get_flag_accessor(PrimitiveType::Face)}}
    , ff_accessor(m.create_accessor<long>(m.m_ff_handle))
    , fe_accessor(m.create_accessor<long>(m.m_fe_handle))
    , fv_accessor(m.create_accessor<long>(m.m_fv_handle))
    , vf_accessor(m.create_accessor<long>(m.m_vf_handle))
    , ef_accessor(m.create_accessor<long>(m.m_ef_handle))
    , hash_accessor(m.get_cell_hash_accessor())
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
        m_incident_face_datas.emplace_back(get_incident_face_data(f.tuple()));
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
};

void TriMesh::TriMeshOperationExecutor::delete_simplices()
{
    for (size_t d = 0; d < simplex_ids_to_delete.size(); ++d) {
        for (const long id : simplex_ids_to_delete[d]) {
            flag_accessors[d].scalar_attribute(id) = 0;
        }
    }
}

void TriMesh::TriMeshOperationExecutor::update_cell_hash()
{
    for (const long& cell_id : cell_ids_to_update_hash) {
        ++hash_accessor.scalar_attribute(cell_id);
    }
}

const std::array<std::vector<long>, 3>
TriMesh::TriMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const TriMesh& m)
{
    const SimplicialComplex sc = SimplicialComplex::open_star(m, Simplex::edge(tuple));
    std::array<std::vector<long>, 3> ids;
    for (const Simplex& s : sc.get_simplices()) {
        ids[get_simplex_dimension(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

const std::array<std::vector<long>, 3>
TriMesh::TriMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const TriMesh& m)
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

/**
 * @brief handling the topology glueing of ear to non-ear face, transfering data from ear-oldface to
 * ear-newface
 *
 * @param ear_fid the ear that will be glued
 * @param new_face_fid
 * @param old_fid the data where the data is transfered from
 * @param eid the edge between two glued faces
 */
void TriMesh::TriMeshOperationExecutor::update_ids_in_ear(
    const long ear_fid,
    const long new_fid,
    const long old_fid,
    const long new_eid)
{
    //         /|
    //   ear  / |
    //       / new_face
    //      /   |
    //      -----
    if (ear_fid < 0) return;

    auto ear_ff = ff_accessor.vector_attribute(ear_fid);
    auto ear_fe = fe_accessor.vector_attribute(ear_fid);
    for (int i = 0; i < 3; ++i) {
        if (ear_ff[i] == old_fid) {
            ear_ff[i] = new_fid;
            ear_fe[i] = new_eid;
            break;
        }
    }

    ef_accessor.scalar_attribute(new_eid) = ear_fid;
}

void TriMesh::TriMeshOperationExecutor::connect_ears()
{
    assert(!m_incident_face_datas.empty());

    //  ---------v2--------
    // |        / \        |
    // | ef0   /   \   ef1 |
    // |      /     \      |
    // |     /       \     |
    // |  ee0         ee1  |
    // |   /   f_old   \   |
    // |  /             \  |
    // | /               \ |
    // v0------ --> ------v1
    // deleting: v0, ee0, f

    for (const auto& face_data : m_incident_face_datas) {
        const long& ef0 = face_data.ears[0].fid;
        const long& ee0 = face_data.ears[0].eid;
        const long& ef1 = face_data.ears[1].fid;
        const long& ee1 = face_data.ears[1].eid;
        const long& f_old = face_data.fid;
        const long& v1 = m_spine_vids[1];
        const long& v2 = face_data.opposite_vid;

        // TODO: should be detected by link condition
        assert(ef0 > -1 || ef1 > -1);
        // check manifoldness
        assert(ef0 != ef1);

        // change face for v2
        long& v2_face = vf_accessor.scalar_attribute(v2);
        // use ef0 if it exists
        v2_face = (ef0 < 0) ? ef1 : ef0;

        ef_accessor.scalar_attribute(ee1) = v2_face;
        vf_accessor.scalar_attribute(v1) = v2_face;

        // change FF and FE for ears
        update_ids_in_ear(ef0, ef1, f_old, ee1);
        update_ids_in_ear(ef1, ef0, f_old, ee1);
    }
};

void TriMesh::TriMeshOperationExecutor::connect_faces_across_spine()
{
    // find the local eid of the spine of the two side of faces
    assert(m_incident_face_datas.size() == 2);
    const long f_old_top = m_incident_face_datas[0].fid;
    const long f0_top = m_incident_face_datas[0].split_f0;
    const long f1_top = m_incident_face_datas[0].split_f1;
    const long f_old_bottom = m_incident_face_datas[1].fid;
    const long f0_bottom = m_incident_face_datas[1].split_f0;
    const long f1_bottom = m_incident_face_datas[1].split_f1;
    auto ff_old_top = ff_accessor.vector_attribute(f_old_top);
    auto ff_old_bottom = ff_accessor.vector_attribute(f_old_bottom);
    assert(m_mesh.capacity(PrimitiveType::Face) > f0_top);
    assert(m_mesh.capacity(PrimitiveType::Face) > f1_top);
    assert(m_mesh.capacity(PrimitiveType::Face) > f0_bottom);
    assert(m_mesh.capacity(PrimitiveType::Face) > f1_bottom);

    // local edge ids are the same for both, f1 and f2
    long local_eid_top = -1;
    long local_eid_bottom = -1;
    for (size_t i = 0; i < 3; ++i) {
        if (ff_old_top[i] == f_old_bottom) {
            local_eid_top = i;
        }
        if (ff_old_bottom[i] == f_old_top) {
            local_eid_bottom = i;
        }
    }
    assert(local_eid_top > -1);
    assert(local_eid_bottom > -1);
    // TODO write test for assumming top and bottom new fids are in right correspondence
    ff_accessor.vector_attribute(f0_top)[local_eid_top] = f0_bottom;
    ff_accessor.vector_attribute(f0_bottom)[local_eid_bottom] = f0_top;
    ff_accessor.vector_attribute(f1_top)[local_eid_top] = f1_bottom;
    ff_accessor.vector_attribute(f1_bottom)[local_eid_bottom] = f1_top;
}

void TriMesh::TriMeshOperationExecutor::replace_incident_face(
    const long v_new,
    const std::vector<long>& spine_eids,
    IncidentFaceData& face_data)
{
    assert(spine_eids.size() == 2);

    // create new faces
    std::vector<long> new_fids = this->request_simplex_indices(PrimitiveType::Face, 2);
    assert(new_fids.size() == 2);

    face_data.split_f0 = new_fids[0];
    face_data.split_f1 = new_fids[1];

    std::vector<long> splitting_edges = this->request_simplex_indices(PrimitiveType::Edge, 1);
    assert(splitting_edges[0] > -1); // TODO: is this assert reasonable at all?

    //  ---------v2--------
    // |        /|\        |
    // | ef0   / | \   ef1 |
    // |      /  |  \      |
    // |     /  oe   \     |
    // |  ee0    |    ee1  |
    // |   /     |     \   |
    // |  /  f0  |  f1  \  |
    // | /       |       \ |
    // v0--se0-v_new-se1--v1
    const long f0 = new_fids[0]; // face 0
    const long f1 = new_fids[1]; // face 1
    const long ef0 = face_data.ears[0].fid; // ear face 0
    const long ef1 = face_data.ears[1].fid; // ear face 1
    const long ee0 = face_data.ears[0].eid; // ear edge 0
    const long ee1 = face_data.ears[1].eid; // ear edge 1
    const long v0 = m_spine_vids[0]; // spine vertex 0
    const long v1 = m_spine_vids[1]; // spine vertex 1
    const long v2 = face_data.opposite_vid; // opposite vertex
    const long se0 = spine_eids[0]; // spine edge 0
    const long se1 = spine_eids[1]; // spine edge 1
    const long oe = splitting_edges[0]; // orthogonal edge
    const long f_old = face_data.fid; // old face

    // f0
    {
        update_ids_in_ear(ef0, f0, f_old, ee0);

        auto fv = fv_accessor.vector_attribute(f0);
        auto fe = fe_accessor.vector_attribute(f0);
        auto ff = ff_accessor.vector_attribute(f0);
        fv = fv_accessor.vector_attribute(f_old);
        fe = fe_accessor.vector_attribute(f_old);
        ff = ff_accessor.vector_attribute(f_old);
        // correct old connectivity
        for (size_t i = 0; i < 3; ++i) {
            if (fe[i] == ee1) {
                ff[i] = f1;
                fe[i] = oe;
            }

            if (fe[i] == m_operating_edge_id) {
                fe[i] = se0;
            }

            if (fv[i] == v1) {
                fv[i] = v_new;
            }
        }
    }
    // f1
    {
        update_ids_in_ear(ef1, f1, f_old, ee1);

        auto fv = fv_accessor.vector_attribute(f1);
        auto fe = fe_accessor.vector_attribute(f1);
        auto ff = ff_accessor.vector_attribute(f1);
        fv = fv_accessor.vector_attribute(f_old);
        fe = fe_accessor.vector_attribute(f_old);
        ff = ff_accessor.vector_attribute(f_old);
        // correct old connectivity
        for (size_t i = 0; i < 3; ++i) {
            if (fe[i] == ee0) {
                ff[i] = f0;
                fe[i] = oe;
            }

            if (fe[i] == m_operating_edge_id) {
                fe[i] = se1;
            }

            if (fv[i] == v0) {
                fv[i] = v_new;
            }
        }
    }

    // assign each edge one face
    ef_accessor.scalar_attribute(ee0) = f0;
    ef_accessor.scalar_attribute(ee1) = f1;
    ef_accessor.scalar_attribute(oe) = f0;
    ef_accessor.scalar_attribute(se0) = f0;
    ef_accessor.scalar_attribute(se1) = f1;
    // assign each vertex one face
    vf_accessor.scalar_attribute(v0) = f0;
    vf_accessor.scalar_attribute(v1) = f1;
    vf_accessor.scalar_attribute(v2) = f0;
    vf_accessor.scalar_attribute(v_new) = f0;

    // face neighbors on the other side of the spine are updated separately

    return;
}

Tuple TriMesh::TriMeshOperationExecutor::split_edge()
{
    if (!m_mesh.multi_mesh_manager.is_parent_mesh()) {
        return split_edge_single_mesh();
    }
    else
    {
        bool is_boundary_on_parent = m_mesh.is_boundary(m_operating_tuple);
        
        std::vector<std::pair<Tuple, Tuple>> operate_tuples_child;
        for (auto child_mesh_ptr : m_mesh.multi_mesh_manager.child_meshes)
        {
            auto tuple_map_handle = m_mesh.multi_mesh_manager.map_to_child_handles[child_mesh_ptr->multi_mesh_manager.child_id()];
            Tuple t_child = MultiMeshManager::map_tuple_between_meshes(m_mesh, *child_mesh_ptr, tuple_map_handle, m_operating_tuple);

            Tuple t_opp_child; // assign a null tuple
            if (!is_boundary_on_parent)
            {
                Tuple t_opp = m_mesh.switch_face(m_operating_tuple);
                t_opp_child = MultiMeshManager::map_tuple_between_meshes(m_mesh, *child_mesh_ptr, tuple_map_handle, t_opp);
            }
          
            operate_tuples_child.push_back(std::make_pair(t_child, t_opp_child));
        }

        // do split on parent_mesh
        Tuple ret_tuple = split_edge_single_mesh();

        for (auto child_mesh_ptr : m_mesh.multi_mesh_manager.child_meshes)
        {
            
            long child_id = child_mesh_ptr->multi_mesh_manager.child_id();
            Tuple t_child = operate_tuples_child[child_id].first;
            Tuple t_opp_child = operate_tuples_child[child_id].second;

            std::vector<long> child_cell_ids_to_update_hash;
            std::vector<std::pair<long,long>> child_new_cell_ids;
            if (child_mesh_ptr->top_simplex_type() == PrimitiveType::Face)
            {
                // this child_mesh is a TriMesh
                TriMesh child_tri_mesh = *std::static_pointer_cast<TriMesh>(child_mesh_ptr);
                if (!t_child.is_null_tuple())
                {
                    TriMesh::TriMeshOperationExecutor executor_child(child_tri_mesh, t_child);
                    executor_child.split_edge();
                    child_cell_ids_to_update_hash = executor_child.cell_ids_to_update_hash;
                    
                    child_new_cell_ids.push_back(std::make_pair(executor_child.m_incident_face_datas[0].split_f1, executor_child.m_incident_face_datas[0].split_f0));

                    if (executor_child.m_incident_face_datas.size() > 1)
                    {
                        child_new_cell_ids.push_back(std::make_pair(executor_child.m_incident_face_datas[1].split_f1, executor_child.m_incident_face_datas[1].split_f0));
                    }                    
                }
                if (!t_opp_child.is_null_tuple() && child_tri_mesh.simplex_is_equal(Simplex::edge(t_child), Simplex::edge(t_opp_child)))
                {
                    TriMesh::TriMeshOperationExecutor executor_child(child_tri_mesh, t_opp_child);
                    executor_child.split_edge();
                    child_cell_ids_to_update_hash.insert(child_cell_ids_to_update_hash.end(), executor_child.cell_ids_to_update_hash.begin(), executor_child.cell_ids_to_update_hash.end());

                    if (child_new_cell_ids.size() == 0)
                    {
                        child_new_cell_ids.push_back(std::make_pair(-1, -1)); // place holder
                    }
                    child_new_cell_ids.push_back(std::make_pair(executor_child.m_incident_face_datas[0].split_f1, executor_child.m_incident_face_datas[0].split_f0));
                }
                // update_hash on new cells
                for (long i = 0; i < long(m_incident_face_datas.size()); i++)
                {
                    auto split_fs = child_new_cell_ids[i];
                    long split_f1 = split_fs.first;
                    long split_f0 = split_fs.second;
                    if (split_f1 == -1) continue;
                    Tuple tuple_child = child_tri_mesh.face_tuple_from_id(split_f1);
                    Tuple tuple_parent = m_mesh.face_tuple_from_id(m_incident_face_datas[i].split_f1);
                    MultiMeshManager::write_tuple_map_attribute(child_tri_mesh.multi_mesh_manager.map_to_parent_handle, child_tri_mesh, tuple_child, tuple_parent);
                    MultiMeshManager::write_tuple_map_attribute(m_mesh.multi_mesh_manager.map_to_child_handles[child_id], m_mesh, tuple_parent, tuple_child);

                    tuple_child = child_tri_mesh.face_tuple_from_id(split_f0);
                    tuple_parent = m_mesh.face_tuple_from_id(m_incident_face_datas[i].split_f0);
                    MultiMeshManager::write_tuple_map_attribute(child_tri_mesh.multi_mesh_manager.map_to_parent_handle, child_tri_mesh, tuple_child, tuple_parent);
                    MultiMeshManager::write_tuple_map_attribute(m_mesh.multi_mesh_manager.map_to_child_handles[child_id], m_mesh, tuple_parent, tuple_child);
                }

                // update_hash on neighboring cells
                for (auto child_cell_id : child_cell_ids_to_update_hash)
                {
                    auto [t_child_old, t_parent_old] = MultiMeshManager::read_tuple_map_attribute(child_tri_mesh.multi_mesh_manager.map_to_parent_handle, child_tri_mesh, child_tri_mesh.tuple_from_id(PrimitiveType::Face, child_cell_id));

                    long child_cell_hash = child_tri_mesh.get_cell_hash_accessor().scalar_attribute(child_cell_id);
                    long parent_cell_hash = m_mesh.get_cell_hash_accessor().scalar_attribute(t_parent_old.m_global_cid);

                    Tuple t_child_new(t_child_old.m_local_vid, t_child_old.m_local_eid, t_child_old.m_local_fid, t_child_old.m_global_cid, child_cell_hash);

                    Tuple t_parent_new(t_parent_old.m_local_vid, t_parent_old.m_local_eid, t_parent_old.m_local_fid, t_parent_old.m_global_cid, parent_cell_hash);

                    MultiMeshManager::write_tuple_map_attribute(child_tri_mesh.multi_mesh_manager.map_to_parent_handle, child_tri_mesh, t_child_new, t_parent_new);

                    MultiMeshManager::write_tuple_map_attribute(m_mesh.multi_mesh_manager.map_to_child_handles[child_id], m_mesh, t_parent_new, t_child_new);

                }
            }
        }

        return ret_tuple;
    }
}

Tuple TriMesh::TriMeshOperationExecutor::split_edge_single_mesh()
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
    return ret;
    // return m_mesh.with_different_cid(m_operating_tuple, m_incident_face_datas[0].split_f0);
}

Tuple TriMesh::TriMeshOperationExecutor::collapse_edge()
{
    if (!m_mesh.multi_mesh_manager.is_parent_mesh()) {
        return collapse_edge_single_mesh();
    }
    else
    {
        bool is_boundary_on_parent = m_mesh.is_boundary(m_operating_tuple);
        
        std::vector<std::pair<Tuple, Tuple>> operate_tuples_child;
        for (auto child_mesh_ptr : m_mesh.multi_mesh_manager.child_meshes)
        {
            auto tuple_map_handle = m_mesh.multi_mesh_manager.map_to_child_handles[child_mesh_ptr->multi_mesh_manager.child_id()];
            Tuple t_child = MultiMeshManager::map_tuple_between_meshes(m_mesh, *child_mesh_ptr, tuple_map_handle, m_operating_tuple);

            Tuple t_opp_child; // assign a null tuple
            if (!is_boundary_on_parent)
            {
                Tuple t_opp = m_mesh.switch_face(m_operating_tuple);
                t_opp_child = MultiMeshManager::map_tuple_between_meshes(m_mesh, *child_mesh_ptr, tuple_map_handle, t_opp);
            }
          
            operate_tuples_child.push_back(std::make_pair(t_child, t_opp_child));
        }
        

        // do collapse on parent_mesh
        Tuple ret_tuple = collapse_edge_single_mesh();

        for (auto child_mesh_ptr : m_mesh.multi_mesh_manager.child_meshes)
        {
            
            long child_id = child_mesh_ptr->multi_mesh_manager.child_id();
            Tuple t_child = operate_tuples_child[child_id].first;
            Tuple t_opp_child = operate_tuples_child[child_id].second;

            std::vector<long> child_cell_ids_to_update_hash;
            if (child_mesh_ptr->top_simplex_type() == PrimitiveType::Face)
            {
                // this child_mesh is a TriMesh
                TriMesh child_tri_mesh = *std::static_pointer_cast<TriMesh>(child_mesh_ptr);
                if (!t_child.is_null_tuple())
                {
                    TriMesh::TriMeshOperationExecutor executor_child(child_tri_mesh, t_child);
                    executor_child.collapse_edge();
                    child_cell_ids_to_update_hash = executor_child.cell_ids_to_update_hash;
                }
                if (!t_opp_child.is_null_tuple() && child_tri_mesh.simplex_is_equal(Simplex::edge(t_child), Simplex::edge(t_opp_child)))
                {
                    TriMesh::TriMeshOperationExecutor executor_child(child_tri_mesh, t_opp_child);
                    executor_child.collapse_edge();
                    child_cell_ids_to_update_hash.insert(child_cell_ids_to_update_hash.end(), executor_child.cell_ids_to_update_hash.begin(), executor_child.cell_ids_to_update_hash.end());
                }
                // update_hash
                for (auto child_cell_id : child_cell_ids_to_update_hash)
                {
                    auto [t_child_old, t_parent_old] = MultiMeshManager::read_tuple_map_attribute(child_tri_mesh.multi_mesh_manager.map_to_parent_handle, child_tri_mesh, child_tri_mesh.tuple_from_id(PrimitiveType::Face, child_cell_id));

                    long child_cell_hash = child_tri_mesh.get_cell_hash_accessor().scalar_attribute(child_cell_id);
                    long parent_cell_hash = m_mesh.get_cell_hash_accessor().scalar_attribute(t_parent_old.m_global_cid);

                    Tuple t_child_new(t_child_old.m_local_vid, t_child_old.m_local_eid, t_child_old.m_local_fid, t_child_old.m_global_cid, child_cell_hash);

                    Tuple t_parent_new(t_parent_old.m_local_vid, t_parent_old.m_local_eid, t_parent_old.m_local_fid, t_parent_old.m_global_cid, parent_cell_hash);

                    MultiMeshManager::write_tuple_map_attribute(child_tri_mesh.multi_mesh_manager.map_to_parent_handle, child_tri_mesh, t_child_new, t_parent_new);

                    MultiMeshManager::write_tuple_map_attribute(m_mesh.multi_mesh_manager.map_to_child_handles[child_id], m_mesh, t_parent_new, t_child_new);

                }
            }
        }

        return ret_tuple;
    }

}


Tuple TriMesh::TriMeshOperationExecutor::collapse_edge_single_mesh()
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
        auto fv = fv_accessor.vector_attribute(fid);
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

    Tuple ret = m_mesh.edge_tuple_from_id(ret_eid);
    if (m_mesh.id_vertex(ret) != ret_vid) {
        ret = m_mesh.switch_vertex(ret);
    }
    assert(m_mesh.id_vertex(ret) == ret_vid);
    if (m_mesh.id_face(ret) != new_tuple_fid) {
        ret = m_mesh.switch_face(ret);
    }
    assert(m_mesh.id_face(ret) == new_tuple_fid);


    update_cell_hash();
    delete_simplices();

    return ret;

    // return a ccw tuple from left ear if it exists, otherwise return a ccw tuple from right ear
    // return m_mesh.tuple_from_id(PrimitiveType::Vertex, v1);
}

std::vector<long> TriMesh::TriMeshOperationExecutor::request_simplex_indices(
    const PrimitiveType type,
    long count)
{
    m_mesh.reserve_attributes(type, m_mesh.capacity(type) + count);

    return m_mesh.request_simplex_indices(type, count);
}

} // namespace wmtk
