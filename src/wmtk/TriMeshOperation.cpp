
#include "TriMeshOperation.hpp"

namespace wmtk {

TriMesh::TriMeshOperationState::PerFaceData TriMesh::TriMeshOperationState::get_per_face_data(
    const Tuple& t)
{
    PerFaceData face_data;
    face_data.deleted_fid = m_mesh.id(t, PF);
    Tuple t1_edge = m_mesh.switch_tuple(t, PE);
    face_data.V_C_id = m_mesh.id(m_mesh.switch_tuple(t1_edge, PV), PV);
    Tuple t2_edge = m_mesh.switch_tuple(t, PV);
    t2_edge = m_mesh.switch_tuple(t2_edge, PE);

    face_data.ears[0] =
        EarGlobalIDs{/*.fid = */ ff_accessor.vector_attribute(t1_edge)(t1_edge.m_local_eid),
                     /*.eid = */ m_mesh.id(t1_edge, PE)};

    face_data.ears[1] =
        EarGlobalIDs{/*.fid = */ ff_accessor.vector_attribute(t2_edge)(t2_edge.m_local_eid),
                     /*.eid = */ m_mesh.id(t2_edge, PE)};

    return face_data;
}

// constructor
TriMesh::TriMeshOperationState::TriMeshOperationState(TriMesh& m)
    : flag_accessors({{m.get_flag_accessor(PrimitiveType::Vertex),
                       m.get_flag_accessor(PrimitiveType::Edge),
                       m.get_flag_accessor(PrimitiveType::Face)}})
    , ff_accessor(m.create_accessor<long>(m.m_ff_handle))
    , fe_accessor(m.create_accessor<long>(m.m_fe_handle))
    , fv_accessor(m.create_accessor<long>(m.m_fv_handle))
    , vf_accessor(m.create_accessor<long>(m.m_vf_handle))
    , ef_accessor(m.create_accessor<long>(m.m_ef_handle))
    , hash_accessor(m.get_cell_hash_accessor())
    , m_mesh(m)
{}

// constructor
TriMesh::TriMeshOperationState::TriMeshOperationState(TriMesh& m, const Tuple& operating_tuple)
    : flag_accessors({{m.get_flag_accessor(PrimitiveType::Vertex),
                       m.get_flag_accessor(PrimitiveType::Edge),
                       m.get_flag_accessor(PrimitiveType::Face)}})
    , ff_accessor(m.create_accessor<long>(m.m_ff_handle))
    , fe_accessor(m.create_accessor<long>(m.m_fe_handle))
    , fv_accessor(m.create_accessor<long>(m.m_fv_handle))
    , vf_accessor(m.create_accessor<long>(m.m_vf_handle))
    , ef_accessor(m.create_accessor<long>(m.m_ef_handle))
    , hash_accessor(m.get_cell_hash_accessor())
    , m_mesh(m)
    , m_operating_tuple(operating_tuple)

{
    end_point_vids[0] = m_mesh.id(m_operating_tuple, PV);
    end_point_vids[1] = m_mesh.id(m_mesh.switch_tuple(m_operating_tuple, PV), PV);

    E_AB_id = m_mesh.id(m_operating_tuple, PE);
    simplices_to_delete.resize(3);

    FaceDatas.emplace_back(get_per_face_data(m_operating_tuple));
    if (!m_mesh.is_boundary(m_operating_tuple)) {
        FaceDatas.emplace_back(get_per_face_data(m_mesh.switch_tuple(m_operating_tuple, PF)));
    }

    // add all cells to update hash
    // Union of all one_ring faces of A,B,C,(C')
    auto C_tuple = m_mesh.switch_tuple(m_mesh.switch_tuple(m_operating_tuple, PE), PV);
    SimplicialComplex SC =
        wmtk::SimplicialComplex::open_star(Simplex(PV, m_operating_tuple), m_mesh);
    SC.unify_with_complex(wmtk::SimplicialComplex::open_star(
        Simplex(PV, m_mesh.switch_tuple(m_operating_tuple, PV)),
        m_mesh));
    SC.unify_with_complex(wmtk::SimplicialComplex::open_star(Simplex(PV, C_tuple), m_mesh));
    if (!m_mesh.is_boundary(m_operating_tuple)) {
        auto C_prime_tuple = m_mesh.switch_tuple(
            m_mesh.switch_tuple(m_mesh.switch_tuple(m_operating_tuple, PF), PE),
            PV);
        SC.unify_with_complex(
            wmtk::SimplicialComplex::open_star(Simplex(PV, C_prime_tuple), m_mesh));
    }
    auto cells_set = SC.get_simplices(PF);
    for (auto cell : cells_set) {
        cells_to_update_hash.push_back(m_mesh.id(cell.tuple(), PF));
    }
};

void TriMesh::TriMeshOperationState::delete_simplices()
{
    for (int d = 0; d < 3; d++) {
        for (long& simplex_id : simplices_to_delete[d]) {
            flag_accessors[d].scalar_attribute(simplex_id) = 1;
        }
    }
}

void TriMesh::TriMeshOperationState::update_cell_hash()
{
    for (const long& cell_id : cells_to_update_hash) {
        hash_accessor.scalar_attribute(cell_id)++;
    }
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
void TriMesh::TriMeshOperationState::glue_ear_to_face(
    const long ear_fid,
    const long new_face_fid,
    const long old_fid,
    const long eid)
{
    if (ear_fid < 0) return;

    auto ear_ff = ff_accessor.vector_attribute(ear_fid);
    auto ear_fe = fe_accessor.vector_attribute(ear_fid);
    for (int i = 0; i < 3; i++) {
        if (ear_ff(i) == old_fid) {
            ear_ff(i) = new_face_fid;
            ear_fe(i) = eid;
            break;
        }
    }
}

void TriMesh::TriMeshOperationState::merge()
{
    simplices_to_delete[0].push_back(end_point_vids[1]);
    simplices_to_delete[1].push_back(E_AB_id);
    for (int face_index = 0; face_index < FaceDatas.size(); face_index++) {
        if (FaceDatas[face_index].ears[0].fid < 0 && FaceDatas[face_index].ears[1].fid < 0) {
            return; // TODO: throw exception, should be detected by link condition
        }

        // change VF for V_A,V_C
        long& vf_a = vf_accessor.scalar_attribute(end_point_vids[0]);
        long& vf_c = vf_accessor.scalar_attribute(FaceDatas[face_index].V_C_id);
        const long f_ear_l = FaceDatas[face_index].ears[0].fid;
        const long f_ear_r = FaceDatas[face_index].ears[1].fid;
        vf_a = (f_ear_l < 0) ? f_ear_r : f_ear_l;
        vf_c = vf_a;

        // change EF for E_AC
        const long e_ac = FaceDatas[face_index].ears[0].eid;
        long& ef_ac = ef_accessor.scalar_attribute(e_ac);
        ef_ac = vf_a;

        // change FF and FE for ears
        glue_ear_to_face(f_ear_l, f_ear_r, FaceDatas[face_index].deleted_fid, e_ac);
        glue_ear_to_face(f_ear_r, f_ear_l, FaceDatas[face_index].deleted_fid, e_ac);

        simplices_to_delete[1].push_back(FaceDatas[face_index].ears[1].eid);
        simplices_to_delete[2].push_back(FaceDatas[face_index].deleted_fid);
    }
};

void TriMesh::collapse_edge(const Tuple& t)
{
    // TODO: check link_cond before collapse
    TriMeshOperationState state(*this, t);

    // get faces in open_star(B)
    auto star_B_f =
        SimplicialComplex::open_star(Simplex(PV, switch_tuple(t, PV)), *this).get_simplices(PF);
    std::vector<long> faces_in_open_star_B_id;
    for (const Simplex& simplex : star_B_f) {
        faces_in_open_star_B_id.push_back(id(simplex.tuple(), PF));
    }

    // merge
    state.merge();

    // change FV for open_star_faces(V_B)
    for (long& f : faces_in_open_star_B_id) {
        for (long index = 0; index < 3; index++) {
            if (state.fv_accessor.vector_attribute(f)[index] == state.end_point_vids[1])
                state.fv_accessor.vector_attribute(f)[index] = state.end_point_vids[0];
            break;
        }
    }

    state.update_cell_hash();
    // delete simplices
    state.delete_simplices();
}

void TriMesh::TriMeshOperationState::glue_new_faces_across_AB(
    const std::array<long, 2> new_fids_top,
    const std::array<long, 2> new_fids_bottom)
{
    // find the local eid of AB of the two side of faces
    long local_eid_top = -1;
    long local_eid_bottom = -1;
    assert(FaceDatas.size() == 2);
    long deleted_fid_top = FaceDatas[0].deleted_fid;
    long deleted_fid_bottom = FaceDatas[1].deleted_fid;
    auto ff_deleted_top = ff_accessor.vector_attribute(deleted_fid_top);
    auto ff_deleted_bottom = ff_accessor.vector_attribute(deleted_fid_bottom);
    assert(m_mesh.capacity(PF) > new_fids_top[0]);
    assert(m_mesh.capacity(PF) > new_fids_top[1]);
    assert(m_mesh.capacity(PF) > new_fids_bottom[0]);
    assert(m_mesh.capacity(PF) > new_fids_bottom[1]);

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

std::array<long, 2> TriMesh::TriMeshOperationState::glue_new_triangle_topology(
    const long new_vid,
    const std::vector<long>& replacement_eids,
    const PerFaceData& face_data)
{
    // create new faces
    std::vector<long> new_fids = this->request_simplex_indices(PrimitiveType::Face, 2);
    assert(new_fids.size() == 2);
    // std::array<long, 2> = {new_fids[0], new_fids[1]};
    // create new edges
    std::vector<long> spine_edge = this->request_simplex_indices(PrimitiveType::Edge, 1);
    assert(spine_edge[0] > -1);
    long spine_eid = spine_edge[0];

    for (int i = 0; i < 2; ++i) {
        const EarGlobalIDs& ear = face_data.ears[i];
        const EarGlobalIDs& other_ear = face_data.ears[(i + 1) % 2];

        const long ear_vid = end_point_vids[i];
        const long other_ear_vid = end_point_vids[(i + 1) % 2];
        const long my_fid = new_fids[i];
        const long other_fid = new_fids[(i + 1) % 2];
        // dummy transfer for new face attributes
        // this copies the deleted FF, FE, FV for two new faces. It handles the case of new to ear
        long deleted_fid = face_data.deleted_fid;
        ff_accessor.vector_attribute(my_fid) = ff_accessor.vector_attribute(deleted_fid);
        fe_accessor.vector_attribute(my_fid) = fe_accessor.vector_attribute(deleted_fid);
        fv_accessor.vector_attribute(my_fid) = fv_accessor.vector_attribute(deleted_fid);

        glue_ear_to_face(ear.fid, my_fid, face_data.deleted_fid, ear.eid);

        auto my_fe = fe_accessor.vector_attribute(my_fid);
        auto my_ff = ff_accessor.vector_attribute(my_fid);
        auto my_fv = fv_accessor.vector_attribute(my_fid);

        // now glue FF, FE, FV of the new-new faces across the spine
        for (int j = 0; j < 3; j++) {
            if (my_fe(j) == other_ear.eid) {
                // FF of the new to new
                my_ff(j) = other_fid;
                // FE of the new to new
                my_fe(j) = spine_eid;
            }

            if (my_fv(j) == other_ear_vid) {
                // FV of the new to new
                my_fv(j) = new_vid;
            }
        }
        // VF of the ear
        vf_accessor.scalar_attribute(ear_vid) = my_fid;

        // EF of the ear
        ef_accessor.scalar_attribute(ear.eid) = my_fid;

        // EF of the new
        ef_accessor.scalar_attribute(replacement_eids[i]) = my_fid;

        // FF of the boundary edge. Set this to the old neighbor for now
        // will handle properly when gluing faces across boundary AB
        // TODO: make sure that we cache this outdated neighbor insted of reading from it.
        // This assumes that the old fid is not re-used
        ff_accessor.vector_attribute(my_fid)(m_operating_tuple.m_local_eid) =
            ff_accessor.vector_attribute(deleted_fid)(m_operating_tuple.m_local_eid);
    }

    // use first new fid as the fid for new vertex, spine edge, and opposing vertex
    // VF of the new vertex
    vf_accessor.scalar_attribute(new_vid) = new_fids[0];
    // EF of the spine
    ef_accessor.scalar_attribute(spine_eid) = new_fids[0];
    // VF of opposing vertex
    vf_accessor.scalar_attribute(face_data.V_C_id) = new_fids[0];

    return {new_fids[0], new_fids[1]};
}

void TriMesh::split_edge(const Tuple& t)
{
    // record the deleted simplices topology attributes
    TriMesh::TriMeshOperationState state(*this, t);
    state.split_edge();
}
void TriMesh::TriMeshOperationState::split_edge()
{
    // delete star(edge)
    SimplicialComplex edge_open_star =
        wmtk::SimplicialComplex::open_star(Simplex(PrimitiveType::Edge, m_operating_tuple), m_mesh);

    for (const auto& simplex_d : edge_open_star.get_simplices()) {
        simplices_to_delete[simplex_d.dimension()].emplace_back(
            m_mesh.id(simplex_d.tuple(), simplex_d.primitive_type()));
    }

    // create new vertex
    std::vector<long> new_vids = request_simplex_indices(PrimitiveType::Vertex, 1);
    assert(new_vids.size() == 1);
    const long new_vid = new_vids[0];

    // create new edges
    std::vector<long> replacement_eids = request_simplex_indices(PrimitiveType::Edge, 2);
    assert(replacement_eids.size() == 2);

    std::vector<std::array<long, 2>> new_fids;
    for (int i = 0; i < FaceDatas.size(); ++i) {
        const PerFaceData face_data = FaceDatas[i];
        // glue the topology
        std::array<long, 2> new_fid_per_face =
            glue_new_triangle_topology(new_vid, replacement_eids, face_data);
        new_fids.emplace_back(new_fid_per_face);
    }
    assert(FaceDatas.size() <= 2);
    // update FF on two sides of ab
    if (FaceDatas.size() > 1) {
        std::array<long, 2> new_fids_top = new_fids[0];
        std::array<long, 2> new_fids_bottom = new_fids[1];

        glue_new_faces_across_AB(new_fids_top, new_fids_bottom);
    }
    update_cell_hash();
    delete_simplices();
}

std::vector<long> TriMesh::TriMeshOperationState::request_simplex_indices(
    const PrimitiveType type,
    long count)
{
    m_mesh.reserve_attributes(type, m_mesh.capacity(type) + count + 1);

    return m_mesh.request_simplex_indices(type, count);
}

} // namespace wmtk
