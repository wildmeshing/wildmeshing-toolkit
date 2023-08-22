#include "TetMeshOperationExecutor.hpp"

namespace wmtk {

TetMesh::TetMeshOperationExecutor::IncidentTetData
TetMesh::TetMeshOperationExecutor::get_incident_tet_data(Tuple t)
{
    //
    // * --------------- * --------------- *
    //   \-_           / | \           _-/
    //    \  EarTet   /  |  \   EarTet  /
    //     \  tid1   /   |   \   tid2  /
    //      \     -_/fid1|fid2\_-     /
    //       \     / --_ | _-- \     /
    //        \   /  __- * -__  \   /
    //         \ /_--    F    --_\ /
    //         X ================= *
    //                   E
    // Operating Tuple: vertex-->X, edge-->E, face-->F


    // make sure that edge and vertex of the tuple is the same
    const SimplicialComplex sc = SimplicialComplex::boundary(m_mesh, Simplex::tetrahedron(t));
    for (const Simplex& s : sc.get_edges()) {
        if (m_mesh.simplex_is_equal(Simplex::edge(t), s)) {
            break;
        }
        t = s.tuple();
    }
    assert(m_mesh.simplex_is_equal(Simplex::edge(t), Simplex::edge(m_operating_tuple)));

    if (!m_mesh.simplex_is_equal(Simplex::vertex(t), Simplex::vertex(m_operating_tuple))) {
        t = m_mesh.switch_vertex(t);
    }
    assert(m_mesh.simplex_is_equal(Simplex::vertex(t), Simplex::vertex(m_operating_tuple)));


    const Tuple ear1_face = m_mesh.switch_face(m_mesh.switch_edge(t));
    const Tuple ear2_face = m_mesh.switch_face(m_mesh.switch_edge(m_mesh.switch_vertex(t)));

    IncidentTetData tet_data;
    tet_data.tid = m_mesh.id_tet(t);
    // TODO: opposite_edge? vertices?

    // accessing ear tet id through TT to make it work also at boundaries
    const long ear1_tid = tt_accessor.vector_attribute(ear1_face)[ear1_face.m_local_fid];
    const long ear2_tid = tt_accessor.vector_attribute(ear2_face)[ear2_face.m_local_fid];

    tet_data.ears[0] = EarTet{ear1_tid, m_mesh.id_face(ear1_face)};
    tet_data.ears[1] = EarTet{ear2_tid, m_mesh.id_face(ear2_face)};

    return tet_data;
}

// constructor
TetMesh::TetMeshOperationExecutor::TetMeshOperationExecutor(
    TetMesh& m,
    const Tuple& operating_tuple)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge), m.get_flag_accessor(PrimitiveType::Face), m.get_flag_accessor(PrimitiveType::Tetrahedron)}}
    , tt_accessor(m.create_accessor<long>(m.m_tt_handle))
    , tf_accessor(m.create_accessor<long>(m.m_tf_handle))
    , te_accessor(m.create_accessor<long>(m.m_te_handle))
    , tv_accessor(m.create_accessor<long>(m.m_tv_handle))
    , vt_accessor(m.create_accessor<long>(m.m_vt_handle))
    , et_accessor(m.create_accessor<long>(m.m_et_handle))
    , ft_accessor(m.create_accessor<long>(m.m_ft_handle))
    , hash_accessor(m.get_cell_hash_accessor())
    , m_mesh(m)
    , m_operating_tuple(operating_tuple)
{
    // store ids of edge and incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(m_mesh.switch_vertex(m_operating_tuple));

    // get the closed star of the edge
    const SimplicialComplex edge_closed_star =
        SimplicialComplex::closed_star(m_mesh, Simplex::edge(operating_tuple));

    // get all tets incident to the edge
    for (const Simplex& t : edge_closed_star.get_tetrahedra()) {
        m_incident_tet_datas.emplace_back(get_incident_tet_data(t.tuple()));
    }

    // update hash on all tets in the two-ring neighborhood
    SimplicialComplex hash_update_region(m);
    for (const Simplex& v : edge_closed_star.get_vertices()) {
        const SimplicialComplex v_closed_star = SimplicialComplex::closed_star(m_mesh, v);
        hash_update_region.unify_with_complex(v_closed_star);
    }
    for (const Simplex& t : hash_update_region.get_tetrahedra()) {
        cell_ids_to_update_hash.push_back(m_mesh.id(t));
    }
}

void TetMesh::TetMeshOperationExecutor::delete_simplices()
{
    for (size_t d = 0; d < simplex_ids_to_delete.size(); ++d) {
        for (const long id : simplex_ids_to_delete[d]) {
            flag_accessors[d].scalar_attribute(id) = 0;
        }
    }
}

void TetMesh::TetMeshOperationExecutor::update_cell_hash()
{
    for (const long& cell_id : cell_ids_to_update_hash) {
        ++hash_accessor.scalar_attribute(cell_id);
    }
}

const std::array<std::vector<long>, 4>
TetMesh::TetMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const TetMesh& m)
{
    const SimplicialComplex sc = SimplicialComplex::open_star(m, Simplex::edge(tuple));
    std::array<std::vector<long>, 4> ids;
    for (const Simplex& s : sc.get_simplices()) {
        ids[get_simplex_dimension(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

const std::array<std::vector<long>, 4>
TetMesh::TetMeshOperationExecutor::get_split_simplices_to_delete(const Tuple& tuple, TetMesh& m)
{
    const SimplicialComplex vertex_open_star =
        SimplicialComplex::open_star(m, Simplex::vertex(tuple));
    const SimplicialComplex edge_closed_star =
        SimplicialComplex::closed_star(m, Simplex::edge(tuple));

    const SimplicialComplex sc =
        SimplicialComplex:: : get_intersection(vertex_open_star, edge_closed_star);

    std::array<std::vector<long>, 4> ids;
    for (const Simplex& s : sc.get_simplices()) {
        ids[get_simplex_dimension(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

const std::array<std::vector<long>, 4>
TetMesh::TetMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const TriMesh& m)
{
    // TODO
}

void TetMesh::TetMeshOperationExecutor::update_ear_connectivity(
    const long ear_tid,
    const long new_tid,
    const long old_tid,
    const long common_fid)
{
    if (ear_tid < 0) return;

    auto ear_tt = tt_accessor.vector_attribute(ear_tid);
    auto ear_tf = tf_accessor.vector_attribute(ear_tid);
    for (int i = 0; i < 4; i++) {
        if (ear_tt(i) == old_tid) {
            ear_tt(i) = new_tid;
            ear_tf(i) = common_fid; // redundant for split
            break;
        }
    }

    ft_accessor.scalar_attribute(common_fid) = ear_fid;
}

Tuple TetMesh::TetMeshOperationExecutor::split_edge()
{
    simplex_ids_to_delete = get_split_simplices_to_delete(m_operating_tuple, m_mesh);

    // create new vertex (center)
    std::vector<long> new_vids = this->request_simplex_indices(PrimitiveType::Vertex, 1);
    assert(new_vids.size() == 1);
    const long v_new = vew_vids[0];

    // create new edges (spline)
    std::vector<long> new_eids = this->request_simplex_indices(PrimitiveType::Edge, 2);
    assert(new_eids.size() == 2);

    /*
        code need to be generalized
    */
    // get incident tets and faces(two cases: loop and boundary)
    std::vector<Tuple> incident_tets;
    std::vector<Tuple> incident_faces;

    // first tet
    /*
          /\\
         /ot\\
        /____\\
    */
    incident_tets.emplace_back(m_operating_tuple);
    incident_faces.emplace_back(m_operating_tuple);

    // direction
    /*
          /\\         /\\
         /ot\\  -->  /  \\  --> ...
        /____\\     /____\\
    */
    // incident face size = incident tet size if loop
    Tuple iter_tuple = m_operating_tuple;
    bool loop_flag = false;
    while (!is_boundary(iter_tuple)) {
        iter_tuple = switch_tuple(iter_tuple, PrimitiveType::Tetrahedron);

        // if no boundary, break;
        if (id_tet(iter_tuple) == id_tet(m_operating_tuple)) {
            loop_flag = true;
            break;
        }

        // switch to another face
        iter_tuple = switch_face(iter_tuple);
        incident_tets.emplace_back(iter_tuple);
        incident_faces.emplace_back(iter_tuple);
    }

    if (!loop_flag) {
        // has boundary case
        // go to an one boundary and start there
        // direction
        /*
              /\\         /\\
             /  \\  -->  /ot\\  --> ...
            /____\\     /____\\
        */
        // incident face size = incident tet size + 1 if boundary

        incident_tets.clear();
        incident_faces.clear();

        // go to the left boundary
        iter_tuple = switch_face(m_operating_tuple);
        while (!is_boundary(iter_tuple)) {
            iter_tuple = switch_face(switch_tuple(iter_tuple, PrimitiveType::Tetrahedron));
        }

        const Tuple last_face_tuple = iter_tuple;
        iter_tuple = switch_face(iter_tuple);

        incident_tets.emplace_back(iter_tuple);
        incident_faces.emplace_back(iter_tuple);

        while (!is_boundary(iter_tuple)) {
            iter_tuple = switch_face(switch_tuple(iter_tuple, PrimitiveType::Tetrahedron));
            incident_tets.emplace_back(iter_tuple);
            incident_faces.emplace_back(iter_tuple);
        }

        incident_faces.emplace_back(last_face_tuple);
    }

    /*
        All incident data collected.
        incident_tets[i] has incident_faces[i] and incident_faces[(i + incident_faces.size()-1) mod
        incident_faces.size()]
    */

    // create new faces and edges
    std::vector<FaceSplitData> new_incident_face_data;
    for (long i = 0; i < incident_faces.size(); i++) {
        std::vector<long> new_fids = this->request_simplex_indices(PrimitiveType::Face, 2);
        std::vector<long> splitting_eids = this->request_simplex_indices(PrimitiveType::Edge, 1);

        FaceSplitData fsd;
        fsd.fid_old = id_face(incident_faces[i]);
        fsd.fid_new_1 = new_fids[0];
        fsd.fid_new_2 = new_fids[1];
        fsd.eid_spine_old = m_operating_edge_id;
        fsd.eid_spine_1 = new_eids[0];
        fsd.eid_spine_2 = new_eids[1];
        fsd.eid_split = splitting_eids[0];
        new_incident_face_data.emplace_back(fsd);
    }

    long incident_face_cnt = new_incident_face_data.size();

    // create new tets
    std::vector<TetSplitData> new_incident_tet_data;
    for (long i = 0; i < incident_tets.size(); i++) {
        std::vector<long> new_tids = this->request_simplex_indices(PrimitiveType::Tetrahedron, 2);
        std::vector<long> split_fids = this->request_simplex_indices(PrimitiveType::Face, 1);

        TetSplitData tsd;
        tsd.tid_old = id_tet(incident_tets[i]);
        tsd.tid_new_1 = new_tids[0];
        tsd.tid_new_2 = new_tids[1];
        tsd.fid_split = split_fids[0];

        // get ears here
        Tuple ear1 = switch_face(switch_edge(incident_tets[i]));
        if (!is_boundary(ear1)) {
            ear1 = switch_tuple(ear1, PrimitiveType::Tetrahedron);
            tsd.ear_tet_1 = EarTet(id_tet(ear1), id_face(ear1));
        } else {
            tsd.ear_tet_1 = EarTet(-1, id_face(ear1));
        }

        Tuple ear2 = switch_face(switch_edge(switch_vertex(incident_tets[i])));
        if (!is_boundary(ear2)) {
            ear2 = switch_tuple(ear2, PrimitiveType::Tetrahedron);
            tsd.ear_tet_2 = EarTet(id_tet(ear2), id_face(ear2));
        } else {
            tsd.ear_tet_2 = EarTet(-1, id_face(ear2));
        }

        tsd.new_face_data[0] =
            new_incident_face_data[(i + incident_face_cnt - 1) % incident_face_cnt];
        tsd.new_face_data[1] = new_incident_face_data[i];
    }

    // update connectivity
    for (long i = 0; i < new_incident_tet_data.size(); i++) {
        // prepare all indices
        const auto& data = new_incident_tet_data[i];
        const long vid_new = v_new;
        // TODO
        // const long v1 =      // m_operating_tuple.vid
        // const long v2 =      // switch_vertex(m_operating_tuple.vid)
        // const long v3 =      // old_face_0 opposite v
        // const long v4 =      // old_face_1 opposite v
        const long e_spline_1 = new_eids[0];
        const long e_spline_2 = new_eids[1];
        const long e_split_1 = data.new_face_data[0].eid_split;
        const long e_split_2 = data.new_face_data[1].eid_split;
        // TODO
        // const long e12 =
        // const long e13 =
        // const long e23 =
        // const long e14 =
        // const long e24 =
        // const long e34 =
        const long f_ear_1 = data.ear_tet_1.eid;
        const long f_ear_2 = data.ear_tet_2.eid;
        const long f1 = data.new_face_data[0].fid_new_1;
        const long f2 = data.new_face_data[0].fid_new_2;
        const long f_old_1 = data.new_face_data[0].fid_old; // f1 + f2
        const long f3 = data.new_face_data[1].fid_new_1;
        const long f4 = data.new_face_data[1].fid_new_2;
        const long f_old_2 = data.new_face_data[1].fid_old; // f3 + f4
        const long f_split = data.fid_split;
        const long t_ear_1 = data.ear_tet_1.tid;
        const long t_ear_2 = data.ear_tet_2.tid;
        const long t1 = data.tid_new_1;
        const long t2 = data.tid_new_2;
        const long t_old = data.tid_old;

        // t1
        {
            // update ear tet 1 (tt, tf)
            update_ear_connectivity(t_ear_1, t1, t_old, f_ear_1);

            auto tt = tt_accessor.vector_attribute(t1);
            auto tf = tf_accessor.vector_attribute(t1);
            auto te = te_accessor.vector_attribute(t1);
            auto tv = tv_accessor.vector_attribute(t1);

            /*
                copy t_old
                v2 --> v_new
                e24 --> e_split2
                e23 --> e_split1
                e12 --> e_spline1
                f_old_1 --> f1
                f_old_2 --> f3
                f_ear_2 --> fsp
                t(f_ear_2) --> t2
                t(f_old_1) --> -1 or tetdata[idx-1].t1
                t(f_old_2) --> -1 or tetdata[idx+1].t1
            */
        }
    }
}


} // namespace wmtk