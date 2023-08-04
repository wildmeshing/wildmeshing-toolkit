
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
    , simplices_to_delete(m)

{
    // store ids of edge and incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(m_mesh.switch_vertex(m_operating_tuple));

    const SimplicialComplex edge_closed_star =
        SimplicialComplex::closed_star(Simplex::edge(operating_tuple), m_mesh);

    // get all faces incident to the edge
    for (const Simplex& f : edge_closed_star.get_faces()) {
        m_incident_face_datas.emplace_back(get_incident_face_data(f.tuple()));
    }

    // update hash on all faces in the two-ring neighborhood
    SimplicialComplex hash_update_region(m);
    for (const Simplex& v : edge_closed_star.get_vertices()) {
        const SimplicialComplex v_closed_star = SimplicialComplex::closed_star(v, m_mesh);
        hash_update_region.unify_with_complex(v_closed_star);
    }
    for (const Simplex& f : hash_update_region.get_faces()) {
        cell_ids_to_update_hash.push_back(m_mesh.id(f));
    }
};

void TriMesh::TriMeshOperationExecutor::delete_simplices()
{
    for (const Simplex& s : simplices_to_delete.get_simplices()) {
        const long d = get_simplex_dimension(s.primitive_type());
        const long id = m_mesh.id(s);
        flag_accessors[d].scalar_attribute(id) = 0;
    }
}

void TriMesh::TriMeshOperationExecutor::update_cell_hash()
{
    for (const long& cell_id : cell_ids_to_update_hash) {
        hash_accessor.scalar_attribute(cell_id)++;
    }
}

const SimplicialComplex TriMesh::TriMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const TriMesh& m)
{
    return SimplicialComplex::open_star(Simplex::edge(tuple), m);
}

const SimplicialComplex TriMesh::TriMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const TriMesh& m)
{
    const SimplicialComplex vertex_open_star =
        SimplicialComplex::open_star(Simplex::vertex(tuple), m);
    const SimplicialComplex edge_closed_star =
        SimplicialComplex::closed_star(Simplex::edge(tuple), m);

    return SimplicialComplex::get_intersection(vertex_open_star, edge_closed_star);
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
void TriMesh::TriMeshOperationExecutor::update_fid_in_ear(
    const long ear_fid,
    const long new_face_fid,
    const long old_fid,
    const long eid)
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
            ear_ff[i] = new_face_fid;
            ear_fe[i] = eid; // TODO: this should have been the same eid before already. Check!
            break;
        }
    }
}

void TriMesh::TriMeshOperationExecutor::merge(const long& new_vid)
{
    simplices_to_delete = get_collapse_simplices_to_delete(m_operating_tuple, m_mesh);

    for (int face_index = 0; face_index < m_incident_face_datas.size(); face_index++) {
        const auto& face_data = m_incident_face_datas[face_index];

        assert(
            face_data.ears[0].fid > -1 ||
            face_data.ears[1].fid > -1); // TODO: should be detected by link condition
        // check manifoldness
        assert(face_data.ears[0].fid != face_data.ears[1].fid);

        // change VF for V_new,V_C
        long& vf_new = vf_accessor.scalar_attribute(new_vid);
        long& vf_c = vf_accessor.scalar_attribute(face_data.opposite_vid);
        const long f_ear_l = face_data.ears[0].fid;
        const long f_ear_r = face_data.ears[1].fid;
        vf_c = (f_ear_l < 0) ? f_ear_r : f_ear_l;
        if (face_index == 0) {
            vf_new = vf_c;
        }

        // change EF for E_AC
        const long e_ac = face_data.ears[0].eid;
        long& ef_ac = ef_accessor.scalar_attribute(e_ac);
        ef_ac = vf_c;

        // change FF and FE for ears
        update_fid_in_ear(f_ear_l, f_ear_r, face_data.fid, e_ac);
        update_fid_in_ear(f_ear_r, f_ear_l, face_data.fid, e_ac);
    }
};

void TriMesh::TriMeshOperationExecutor::glue_new_faces_across_AB(
    const std::array<long, 2> new_fids_top,
    const std::array<long, 2> new_fids_bottom)
{
    // find the local eid of AB of the two side of faces
    long local_eid_top = -1;
    long local_eid_bottom = -1;
    assert(m_incident_face_datas.size() == 2);
    long deleted_fid_top = m_incident_face_datas[0].fid;
    long deleted_fid_bottom = m_incident_face_datas[1].fid;
    auto ff_deleted_top = ff_accessor.vector_attribute(deleted_fid_top);
    auto ff_deleted_bottom = ff_accessor.vector_attribute(deleted_fid_bottom);
    assert(m_mesh.capacity(PrimitiveType::Face) > new_fids_top[0]);
    assert(m_mesh.capacity(PrimitiveType::Face) > new_fids_top[1]);
    assert(m_mesh.capacity(PrimitiveType::Face) > new_fids_bottom[0]);
    assert(m_mesh.capacity(PrimitiveType::Face) > new_fids_bottom[1]);

    for (int i = 0; i < 3; i++) {
        if (ff_deleted_top(i) == deleted_fid_bottom) {
            local_eid_top = i;
        }
        if (ff_deleted_bottom(i) == deleted_fid_top) {
            local_eid_bottom = i;
        }
    }
    assert(local_eid_top > -1);
    assert(local_eid_bottom > -1);
    // TODO write test for assumming top and bottom new fids are in right correspondence
    ff_accessor.vector_attribute(new_fids_top[0])[local_eid_top] = new_fids_bottom[0];
    ff_accessor.vector_attribute(new_fids_bottom[0])[local_eid_bottom] = new_fids_top[0];
    ff_accessor.vector_attribute(new_fids_top[1])[local_eid_top] = new_fids_bottom[1];
    ff_accessor.vector_attribute(new_fids_bottom[1])[local_eid_bottom] = new_fids_top[1];
}

std::array<long, 2> TriMesh::TriMeshOperationExecutor::glue_new_triangle_topology(
    const long v_new,
    const std::vector<long>& spine_eids,
    IncidentFaceData& face_data)
{
    assert(spine_eids.size() == 2);

    // create new faces
    std::vector<long> new_fids = this->request_simplex_indices(PrimitiveType::Face, 2);
    assert(new_fids.size() == 2);

    face_data.f0 = new_fids[0];
    face_data.f1 = new_fids[1];

    std::vector<long> splitting_edges = this->request_simplex_indices(PrimitiveType::Edge, 1);
    assert(splitting_edges[0] > -1); // TODO: is this assert reasonable at all?
    const long splitting_eid = splitting_edges[0];

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
    const long f0 = new_fids[0];
    const long f1 = new_fids[1];
    const long ef0 = face_data.ears[0].fid;
    const long ef1 = face_data.ears[1].fid;
    const long ee0 = face_data.ears[0].eid;
    const long ee1 = face_data.ears[1].eid;
    const long v0 = m_spine_vids[0];
    const long v1 = m_spine_vids[1];
    const long v2 = face_data.opposite_vid;
    const long se0 = spine_eids[0];
    const long se1 = spine_eids[1];
    const long oe = splitting_eid;
    const long f_old = face_data.fid;

    // f0
    {
        update_fid_in_ear(ef0, f0, f_old, ee0);

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
        update_fid_in_ear(ef1, f1, f_old, ee1);

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


    return {new_fids[0], new_fids[1]};
}

Tuple TriMesh::TriMeshOperationExecutor::split_edge()
{
    simplices_to_delete = get_split_simplices_to_delete(m_operating_tuple, m_mesh);

    // create new vertex (center)
    std::vector<long> new_vids = this->request_simplex_indices(PrimitiveType::Vertex, 1);
    assert(new_vids.size() == 1);
    const long new_vid = new_vids[0];

    // create new edges (spine)
    std::vector<long> new_eids = this->request_simplex_indices(PrimitiveType::Edge, 2);
    assert(new_eids.size() == 2);

    std::vector<std::array<long, 2>> new_fids;
    for (IncidentFaceData& face_data : m_incident_face_datas) {
        // glue the topology
        const std::array<long, 2> new_fids_per_face =
            glue_new_triangle_topology(new_vid, new_eids, face_data);
        new_fids.emplace_back(new_fids_per_face);
    }
    assert(m_incident_face_datas.size() <= 2);
    // update FF on two sides of ab
    if (m_incident_face_datas.size() > 1) {
        const std::array<long, 2>& new_fids_top = new_fids[0];
        const std::array<long, 2>& new_fids_bottom = new_fids[1];

        glue_new_faces_across_AB(new_fids_top, new_fids_bottom);
    }
    update_cell_hash();
    delete_simplices();
    // return Tuple new_fid, new_vid that points
    return m_mesh.with_different_cid(m_operating_tuple, new_fids[0][0]);
}

std::vector<long> TriMesh::TriMeshOperationExecutor::request_simplex_indices(
    const PrimitiveType type,
    long count)
{
    m_mesh.reserve_attributes(type, m_mesh.capacity(type) + count);

    return m_mesh.request_simplex_indices(type, count);
}

} // namespace wmtk
