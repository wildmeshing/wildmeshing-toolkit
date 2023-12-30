#include "TetMeshOperationExecutor.hpp"
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/boundary.hpp>
#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>

namespace wmtk {
namespace {
constexpr static PrimitiveType PE = PrimitiveType::Edge;
constexpr static PrimitiveType PF = PrimitiveType::Face;
} // namespace

std::tuple<std::vector<Tuple>, std::vector<Tuple>>
TetMesh::TetMeshOperationExecutor::get_incident_tets_and_faces(Tuple t)
{
    std::vector<Tuple> incident_tets, incident_faces;

    incident_tets.emplace_back(t);
    incident_faces.emplace_back(t);

    // direction
    /*
          /\\         /\\
         /ot\\  -->  /  \\  --> ...
        /____\\     /____\\
    */
    // incident face size = incident tet size if loop
    Tuple iter_tuple = t;

    bool loop_flag = false;

    while (!m_mesh.is_boundary_face(iter_tuple)) {
        iter_tuple = m_mesh.switch_tetrahedron(iter_tuple);

        // if no boundary, break;
        if (m_mesh.id_tet(iter_tuple) == m_mesh.id_tet(t)) {
            loop_flag = true;
            break;
        }

        // switch to another face
        iter_tuple = m_mesh.switch_face(iter_tuple);
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
        iter_tuple = m_mesh.switch_face(t);
        while (!m_mesh.is_boundary_face(iter_tuple)) {
            iter_tuple = m_mesh.switch_face(m_mesh.switch_tetrahedron(iter_tuple));
        }

        const Tuple last_face_tuple = iter_tuple;
        iter_tuple = m_mesh.switch_face(iter_tuple);

        incident_tets.emplace_back(iter_tuple);
        incident_faces.emplace_back(iter_tuple);

        while (!m_mesh.is_boundary_face(iter_tuple)) {
            iter_tuple = m_mesh.switch_face(m_mesh.switch_tetrahedron(iter_tuple));
            incident_tets.emplace_back(iter_tuple);
            incident_faces.emplace_back(iter_tuple);
        }

        incident_faces.emplace_back(last_face_tuple);
    }

    return {incident_tets, incident_faces};
}

// TODO: This is not used
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
    const simplex::SimplexCollection sc =
        simplex::boundary(m_mesh, simplex::Simplex::tetrahedron(t));
    for (const simplex::Simplex& s : sc.simplex_vector(PrimitiveType::Edge)) {
        if (simplex::utils::SimplexComparisons::equal(m_mesh, simplex::Simplex::edge(t), s)) {
            break;
        }
        t = s.tuple();
    }
    assert(simplex::utils::SimplexComparisons::equal(
        m_mesh,
        simplex::Simplex::edge(t),
        simplex::Simplex::edge(m_operating_tuple)));

    if (!simplex::utils::SimplexComparisons::equal(
            m_mesh,
            simplex::Simplex::vertex(t),
            simplex::Simplex::vertex(m_operating_tuple))) {
        t = m_mesh.switch_vertex(t);
    }
    assert(simplex::utils::SimplexComparisons::equal(
        m_mesh,
        simplex::Simplex::vertex(t),
        simplex::Simplex::vertex(m_operating_tuple)));


    const Tuple ear1_face = m_mesh.switch_face(m_mesh.switch_edge(t));
    const Tuple ear2_face = m_mesh.switch_face(m_mesh.switch_edge(m_mesh.switch_vertex(t)));

    IncidentTetData tet_data;
    tet_data.tid = m_mesh.id_tet(t);
    // TODO: opposite_edge? vertices?

    // accessing ear tet id through TT to make it work also at boundaries
    const long ear1_tid = tt_accessor.const_vector_attribute(ear1_face)[ear1_face.m_local_fid];
    const long ear2_tid = tt_accessor.const_vector_attribute(ear2_face)[ear2_face.m_local_fid];

    tet_data.ears[0] = EarTet{ear1_tid, m_mesh.id_face(ear1_face)};
    tet_data.ears[1] = EarTet{ear2_tid, m_mesh.id_face(ear2_face)};

    return tet_data;
}

// constructor
TetMesh::TetMeshOperationExecutor::TetMeshOperationExecutor(
    TetMesh& m,
    const Tuple& operating_tuple,
    Accessor<long>& hash_acc)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge), m.get_flag_accessor(PrimitiveType::Face), m.get_flag_accessor(PrimitiveType::Tetrahedron)}}
    , tt_accessor(m.create_accessor<long>(m.m_tt_handle))
    , tf_accessor(m.create_accessor<long>(m.m_tf_handle))
    , te_accessor(m.create_accessor<long>(m.m_te_handle))
    , tv_accessor(m.create_accessor<long>(m.m_tv_handle))
    , vt_accessor(m.create_accessor<long>(m.m_vt_handle))
    , et_accessor(m.create_accessor<long>(m.m_et_handle))
    , ft_accessor(m.create_accessor<long>(m.m_ft_handle))
    , hash_accessor(hash_acc)
    , m_mesh(m)
{
    m_operating_tuple = operating_tuple;
    // store ids of edge and incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(m_mesh.switch_vertex(m_operating_tuple));
    m_operating_face_id = m_mesh.id_face(m_operating_tuple);
    m_operating_tet_id = m_mesh.id_tet(m_operating_tuple);


    // get the closed star of the edge
    const simplex::SimplexCollection edge_closed_star =
        simplex::closed_star(m_mesh, simplex::Simplex::edge(operating_tuple));

    // get all tets incident to the edge
    // TODO: having another implementation, remove this here
    // for (const simplex::Simplex& t : edge_closed_star.get_tetrahedra()) {
    //     m_incident_tet_datas.emplace_back(get_incident_tet_data(t.tuple()));
    // }

    // update hash on all tets in the two-ring neighborhood
    simplex::SimplexCollection hash_update_region(m);
    for (const simplex::Simplex& v : edge_closed_star.simplex_vector(PrimitiveType::Vertex)) {
        const simplex::SimplexCollection v_closed_star = simplex::closed_star(m_mesh, v);
        hash_update_region.add(v_closed_star);
    }
    hash_update_region.sort_and_clean();
    for (const simplex::Simplex& t :
         hash_update_region.simplex_vector(PrimitiveType::Tetrahedron)) {
        cell_ids_to_update_hash.push_back(m_mesh.id(t));
    }
}

void TetMesh::TetMeshOperationExecutor::delete_simplices()
{
    for (size_t d = 0; d < simplex_ids_to_delete.size(); ++d) {
        for (const long id : simplex_ids_to_delete[d]) {
            flag_accessors[d].index_access().scalar_attribute(id) = 0; // TODO: reset single bit
        }
    }
}

void TetMesh::TetMeshOperationExecutor::update_cell_hash()
{
    m_mesh.update_cell_hashes(cell_ids_to_update_hash, hash_accessor);
}

const std::array<std::vector<long>, 4>
TetMesh::TetMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const TetMesh& m)
{
    const simplex::SimplexCollection sc = simplex::open_star(m, simplex::Simplex::edge(tuple));
    std::array<std::vector<long>, 4> ids;
    for (const simplex::Simplex& s : sc) {
        ids[get_primitive_type_id(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

const std::array<std::vector<long>, 4>
TetMesh::TetMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const TetMesh& m)
{
    const simplex::SimplexCollection vertex_open_star =
        simplex::open_star(m, simplex::Simplex::vertex(tuple));
    const simplex::SimplexCollection edge_closed_star =
        simplex::closed_star(m, simplex::Simplex::edge(tuple));

    const simplex::SimplexCollection sc =
        simplex::SimplexCollection::get_intersection(vertex_open_star, edge_closed_star);

    std::array<std::vector<long>, 4> ids;
    for (const simplex::Simplex& s : sc) {
        ids[get_primitive_type_id(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

void TetMesh::TetMeshOperationExecutor::update_ear_connectivity(
    const long ear_tid,
    const long new_tid,
    const long old_tid,
    const long common_fid)
{
    if (ear_tid < 0) return;

    auto ear_tt = tt_accessor.index_access().vector_attribute(ear_tid);
    auto ear_tf = tf_accessor.index_access().vector_attribute(ear_tid);
    for (int i = 0; i < 4; ++i) {
        if (ear_tt(i) == old_tid) {
            ear_tt(i) = new_tid;
            ear_tf(i) = common_fid; // redundant for split
            break;
        }
    }

    ft_accessor.index_access().scalar_attribute(common_fid) = ear_tid;
}

void TetMesh::TetMeshOperationExecutor::split_edge()
{
    simplex_ids_to_delete = get_split_simplices_to_delete(m_operating_tuple, m_mesh);

    // create new vertex (center)
    std::vector<long> new_vids = this->request_simplex_indices(PrimitiveType::Vertex, 1);
    assert(new_vids.size() == 1);
    const long v_new = new_vids[0];
    m_split_new_vid = v_new;

    // create new edges (spline)
    std::vector<long> new_eids = this->request_simplex_indices(PrimitiveType::Edge, 2);
    assert(new_eids.size() == 2);
    std::copy(new_eids.begin(), new_eids.end(), m_split_new_spine_eids.begin());

    // get incident tets and faces(two cases: loop and boundary)
    // auto incident_tets_and_faces = get_incident_tets_and_faces(m_operating_tuple);
    // const auto& incident_tets = incident_tets_and_faces[0];
    // const auto& incident_faces = incident_tets_and_faces[1];

    const auto [incident_tets, incident_faces] = get_incident_tets_and_faces(m_operating_tuple);
    const bool loop_flag = (incident_tets.size() == incident_faces.size());


    // create new faces and edges
    std::vector<FaceSplitData> new_incident_face_data;
    for (long i = 0; i < incident_faces.size(); ++i) {
        std::vector<long> new_fids = this->request_simplex_indices(PrimitiveType::Face, 2);
        std::vector<long> splitting_eids = this->request_simplex_indices(PrimitiveType::Edge, 1);

        FaceSplitData fsd;
        fsd.fid_old = m_mesh.id_face(incident_faces[i]);
        fsd.fid_new_1 = new_fids[0];
        fsd.fid_new_2 = new_fids[1];
        fsd.eid_spine_old = m_operating_edge_id;
        fsd.eid_spine_1 = new_eids[0]; // redundant
        fsd.eid_spine_2 = new_eids[1]; // redundant
        fsd.eid_split = splitting_eids[0]; // redundant
        new_incident_face_data.emplace_back(fsd);
    }


    long incident_face_cnt = new_incident_face_data.size();

    // create new tets
    std::vector<TetSplitData> tet_split_data;
    for (long i = 0; i < incident_tets.size(); ++i) {
        std::vector<long> new_tids = this->request_simplex_indices(PrimitiveType::Tetrahedron, 2);
        std::vector<long> split_fids = this->request_simplex_indices(PrimitiveType::Face, 1);

        TetSplitData tsd;
        tsd.tid_old = m_mesh.id_tet(incident_tets[i]);
        tsd.tid_new_1 = new_tids[0];
        tsd.tid_new_2 = new_tids[1];
        tsd.fid_split = split_fids[0];

        // get ears here
        Tuple ear1 = m_mesh.switch_face(m_mesh.switch_edge(incident_tets[i]));
        if (!m_mesh.is_boundary_face(ear1)) {
            ear1 = m_mesh.switch_tuple(ear1, PrimitiveType::Tetrahedron);
            tsd.ear_tet_1 = EarTet{m_mesh.id_tet(ear1), m_mesh.id_face(ear1)};
        } else {
            tsd.ear_tet_1 = EarTet{-1, m_mesh.id_face(ear1)};
        }

        Tuple ear2 = m_mesh.switch_face(m_mesh.switch_edge(m_mesh.switch_vertex(incident_tets[i])));
        if (!m_mesh.is_boundary_face(ear2)) {
            ear2 = m_mesh.switch_tuple(ear2, PrimitiveType::Tetrahedron);
            tsd.ear_tet_2 = EarTet{m_mesh.id_tet(ear2), m_mesh.id_face(ear2)};
        } else {
            tsd.ear_tet_2 = EarTet{-1, m_mesh.id_face(ear2)};
        }

        tsd.new_face_data[0] =
            new_incident_face_data[(i + incident_face_cnt - 1) % incident_face_cnt];
        tsd.new_face_data[1] = new_incident_face_data[i];

        tsd.v1 = m_mesh.id_vertex(incident_tets[i]); // redundant
        tsd.v2 = m_mesh.id_vertex(m_mesh.switch_vertex(incident_tets[i])); // redundant
        tsd.v3 = m_mesh.id_vertex(m_mesh.switch_vertex(
            m_mesh.switch_edge(m_mesh.switch_face(incident_tets[i])))); // put in face
        tsd.v4 = m_mesh.id_vertex(
            m_mesh.switch_vertex(m_mesh.switch_edge(incident_tets[i]))); // put in face rename

        tsd.e12 = m_mesh.id_edge(incident_tets[i]); // redundant
        tsd.e14 = m_mesh.id_edge(m_mesh.switch_edge(incident_tets[i])); // face 1 ear 1 edge
        tsd.e24 = m_mesh.id_edge(
            m_mesh.switch_edge(m_mesh.switch_vertex(incident_tets[i]))); // face 2 ear 2 edge
        tsd.e13 = m_mesh.id_edge(
            m_mesh.switch_edge(m_mesh.switch_face(incident_tets[i]))); // face 1 ear 1 edge
        tsd.e23 = m_mesh.id_edge(m_mesh.switch_edge(
            m_mesh.switch_face(m_mesh.switch_vertex(incident_tets[i])))); // face 2 ear 1 edge
        tsd.e34 = m_mesh.id_edge(m_mesh.switch_edge(m_mesh.switch_vertex(
            m_mesh.switch_face(m_mesh.switch_edge(incident_tets[i]))))); // opposite edge

        tet_split_data.emplace_back(tsd);
    }


    // local ids for return tuple
    long return_local_vid = -1;
    long return_local_eid = -1;
    long return_local_fid = -1;
    long return_tid = -1;

    // these are used only for assertions
#ifndef NDEBUG
    long return_fid = -1;
    long return_split_fid = -1;
#endif

    // update connectivity
    for (long i = 0; i < tet_split_data.size(); ++i) {
        // prepare all indices
        const auto& data = tet_split_data[i];
        const long vid_new = v_new;
        const long v1 = data.v1; // m_operating_tuple.vid
        const long v2 = data.v2; // switch_vertex(m_operating_tuple)
        const long v3 = data.v3; // f_old_1 opposite v
        const long v4 = data.v4; // f_old_2 opposite v
        const long e_spline_1 = new_eids[0];
        const long e_spline_2 = new_eids[1];
        const long e_split_1 = data.new_face_data[0].eid_split;
        const long e_split_2 = data.new_face_data[1].eid_split;
        const long e12 = data.e12;
        const long e13 = data.e13;
        const long e23 = data.e23;
        const long e14 = data.e14;
        const long e24 = data.e24;
        const long e34 = data.e34;
        const long f_ear_1 = data.ear_tet_1.fid;
        const long f_ear_2 = data.ear_tet_2.fid;
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
        long t_f1; // prev t1
        long t_f2; // prev t2
        long t_f3; // next t1
        long t_f4; // next t2

        // for return tuple
        // return_flag == true means this is the tet for the tuple to return

        bool return_flag = false;
        if (t_old == m_operating_tet_id) {
            return_tid = t2;

            logger().trace("split fid is {}", f_split);
            logger().trace("fids {} {} are joined by edge {}", f3, f4, e_split_2);
#ifndef NDEBUG
            return_fid = f4;
            return_split_fid = f_split;
#endif
            return_flag = true;
        }
        long prev_index = (i - 1 + tet_split_data.size()) % tet_split_data.size();
        long next_index = (i + 1 + tet_split_data.size()) % tet_split_data.size();

        if (loop_flag) {
            t_f1 = tet_split_data[prev_index].tid_new_1;
            t_f2 = tet_split_data[prev_index].tid_new_2;
            t_f3 = tet_split_data[next_index].tid_new_1;
            t_f4 = tet_split_data[next_index].tid_new_2;
        } else {
            if (tet_split_data.size() == 1) {
                t_f1 = -1;
                t_f2 = -1;
                t_f3 = -1;
                t_f4 = -1;
            } else {
                if (i == 0) {
                    // no prev
                    t_f1 = -1;
                    t_f2 = -1;
                } else {
                    t_f1 = tet_split_data[prev_index].tid_new_1;
                    t_f2 = tet_split_data[prev_index].tid_new_2;
                }
                if (i == tet_split_data.size() - 1) {
                    // no next
                    t_f3 = -1;
                    t_f4 = -1;
                } else {
                    t_f3 = tet_split_data[next_index].tid_new_1;
                    t_f4 = tet_split_data[next_index].tid_new_2;
                }
            }
        }

        // t1
        {
            // update ear tet 1 (tt, tf)
            update_ear_connectivity(t_ear_1, t1, t_old, f_ear_1);

            auto tt = tt_accessor.index_access().vector_attribute(t1);
            auto tf = tf_accessor.index_access().vector_attribute(t1);
            auto te = te_accessor.index_access().vector_attribute(t1);
            auto tv = tv_accessor.index_access().vector_attribute(t1);

            /*
                copy t_old
                v2 --> v_new
                e24 --> e_split2
                e23 --> e_split1
                e12 --> e_spline1
                f_old_1 --> f1
                f_old_2 --> f3
                f_ear_2 --> fsp
                t(f_ear_2) t_ear_2 --> t2
                t(f_old_1) --> -1 or tetdata[idx-1].t1
                t(f_old_2) --> -1 or tetdata[idx+1].t1
            */
            // copy t_old
            tt = tt_accessor.index_access().vector_attribute(t_old);
            tf = tf_accessor.index_access().vector_attribute(t_old);
            te = te_accessor.index_access().vector_attribute(t_old);
            tv = tv_accessor.index_access().vector_attribute(t_old);

            // get ids for return tuple
            if (return_flag) {
                // vertex and face
                for (int k = 0; k < 4; ++k) {
                    // vertex
                    if (tv(k) == m_spine_vids[0]) {
                        return_local_vid = k;
                    }

                    // face
                    if (tf(k) == m_operating_face_id) {
                        return_local_fid = k;
                    }
                }

                // edge
                for (int k = 0; k < 6; ++k) {
                    if (te(k) == e12) {
                        return_local_eid = k;
                        break;
                    }
                }
            }


            for (size_t k = 0; k < 4; ++k) {
                // vertices
                if (tv(k) == v2) {
                    tv(k) = vid_new;
                }

                // faces and tets
                if (tf(k) == f_old_1) {
                    tf(k) = f1;
                    tt(k) = t_f1;
                }
                if (tf(k) == f_old_2) {
                    tf(k) = f3;
                    tt(k) = t_f3;
                }
                if (tf(k) == f_ear_2) {
                    tf(k) = f_split;
                    tt(k) = t2;
                }
            }

            for (size_t k = 0; k < 6; ++k) {
                // edges
                if (te(k) == e24) {
                    te(k) = e_split_2;
                }
                if (te(k) == e23) {
                    te(k) = e_split_1;
                }
                if (te(k) == e12) {
                    te(k) = e_spline_1;
                }
            }
        }

        // t2
        {
            // update ear tet 2 (tt, tf)
            update_ear_connectivity(t_ear_2, t2, t_old, f_ear_2);

            auto tt = tt_accessor.index_access().vector_attribute(t2);
            auto tf = tf_accessor.index_access().vector_attribute(t2);
            auto te = te_accessor.index_access().vector_attribute(t2);
            auto tv = tv_accessor.index_access().vector_attribute(t2);

            /*
                copy t_old
                v1 --> v_new
                e14 --> e_split2
                e13 --> e_split1
                e12 --> e_spline2
                f_old_1 --> f2
                f_old_2 --> f4
                f_ear_1 --> fsp
                t(f_ear_1) t_ear_1 --> t1
                t(f_old_1) --> -1 or tetdata[idx-1].t2
                t(f_old_2) --> -1 or tetdata[idx+1].t2
            */
            // copy t_old
            tt = tt_accessor.index_access().const_vector_attribute(t_old);
            tf = tf_accessor.index_access().const_vector_attribute(t_old);
            te = te_accessor.index_access().const_vector_attribute(t_old);
            tv = tv_accessor.index_access().const_vector_attribute(t_old);
            for (size_t k = 0; k < 4; ++k) {
                // vertices
                if (tv(k) == v1) {
                    tv(k) = vid_new;
                }

                // faces and tets
                if (tf(k) == f_old_1) {
                    tf(k) = f2;
                    tt(k) = t_f2;
                }
                if (tf(k) == f_old_2) {
                    tf(k) = f4;
                    tt(k) = t_f4;
                }
                if (tf(k) == f_ear_1) {
                    tf(k) = f_split;
                    tt(k) = t1;
                }
            }

            for (size_t k = 0; k < 6; ++k) {
                // edges
                if (te(k) == e14) {
                    te(k) = e_split_2;
                }
                if (te(k) == e13) {
                    te(k) = e_split_1;
                }
                if (te(k) == e12) {
                    te(k) = e_spline_2;
                }
            }
        }

        // assign each face one tet
        ft_accessor.index_access().scalar_attribute(f_ear_1) = t1;
        ft_accessor.index_access().scalar_attribute(f_ear_2) = t2;
        ft_accessor.index_access().scalar_attribute(f1) = t1;
        ft_accessor.index_access().scalar_attribute(f2) = t2;
        ft_accessor.index_access().scalar_attribute(f3) = t1;
        ft_accessor.index_access().scalar_attribute(f4) = t2;
        ft_accessor.index_access().scalar_attribute(f_split) = t1;

        // assign each edge one tet
        et_accessor.index_access().scalar_attribute(e13) = t1;
        et_accessor.index_access().scalar_attribute(e23) = t2;
        et_accessor.index_access().scalar_attribute(e14) = t1;
        et_accessor.index_access().scalar_attribute(e24) = t2;
        et_accessor.index_access().scalar_attribute(e34) = t1;
        et_accessor.index_access().scalar_attribute(e_spline_1) = t1;
        et_accessor.index_access().scalar_attribute(e_spline_2) = t2;
        et_accessor.index_access().scalar_attribute(e_split_1) = t1;
        et_accessor.index_access().scalar_attribute(e_split_2) = t1;

        // assign each vertex one tet
        vt_accessor.index_access().scalar_attribute(v1) = t1;
        vt_accessor.index_access().scalar_attribute(v2) = t2;
        vt_accessor.index_access().scalar_attribute(v3) = t1;
        vt_accessor.index_access().scalar_attribute(v4) = t1;
        vt_accessor.index_access().scalar_attribute(vid_new) = t1;
    }


    // update hash and delete simplices
    update_cell_hash();
    delete_simplices();

    // Tuple ret = m_mesh.edge_tuple_from_id(new_eids[0]);
    // std::cout << "return_tid: " << return_tid << std::endl;
    // std::cout << "return_local_vid: " << return_local_vid << std::endl;
    // std::cout << "return_local_eid: " << return_local_eid << std::endl;
    // std::cout << "return_local_fid: " << return_local_fid << std::endl;


    assert(return_tid > -1);
    assert(return_local_vid > -1);
    assert(return_local_eid > -1);
    assert(return_local_fid > -1);

    assert(return_local_vid == utils::TupleInspector::local_vid(m_operating_tuple));
    assert(return_local_eid == utils::TupleInspector::local_eid(m_operating_tuple));
    assert(return_local_fid == utils::TupleInspector::local_fid(m_operating_tuple));
    const long return_tet_hash = hash_accessor.index_access().scalar_attribute(return_tid);
    m_output_tuple =
        Tuple(return_local_vid, return_local_eid, return_local_fid, return_tid, return_tet_hash);

    assert(m_split_new_vid == m_mesh.id(simplex::Simplex::vertex(m_output_tuple)));
    assert(m_split_new_spine_eids[1] == m_mesh.id(simplex::Simplex::edge(m_output_tuple)));
    assert(return_fid == m_mesh.id(simplex::Simplex::face(m_output_tuple)));
    assert(return_tid == m_mesh.id(simplex::Simplex::tetrahedron(m_output_tuple)));

    logger().trace(
        "split fid is {}",
        m_mesh.id(simplex::Simplex::face(m_mesh.switch_tuples(m_output_tuple, {PE, PF}))));
    // assert(m_mesh.id(simplex::Simplex::edge(m_mesh.switch_tuples(m_output_tuple, {PE}))) =
    // return_face_spine_eid);
    assert(
        m_mesh.id(simplex::Simplex::face(m_mesh.switch_tuples(m_output_tuple, {PE, PF}))) ==
        return_split_fid);
    assert(!m_mesh.is_boundary_face(m_mesh.switch_tuples(m_output_tuple, {PE, PF})));
}

void TetMesh::TetMeshOperationExecutor::collapse_edge()
{
    simplex_ids_to_delete = get_collapse_simplices_to_delete(m_operating_tuple, m_mesh);

    // collect star before changing connectivity
    // update all tv's after other updates
    const simplex::SimplexCollection v1_star =
        simplex::closed_star(m_mesh, simplex::Simplex::vertex(m_operating_tuple));

    // collect incident tets and their ears
    // loop case and boundary case
    // auto incident_tets_and_faces = get_incident_tets_and_faces(m_operating_tuple);
    // const auto& incident_tets = incident_tets_and_faces[0];

    auto [incident_tets, incident_faces] = get_incident_tets_and_faces(m_operating_tuple);


    for (const Tuple& tet : incident_tets) {
        TetCollapseData tcd;
        tcd.tid_old = m_mesh.id_tet(tet);

        // get ears
        Tuple ear1 = m_mesh.switch_face(m_mesh.switch_edge(tet));
        if (!m_mesh.is_boundary_face(ear1)) {
            ear1 = m_mesh.switch_tuple(ear1, PrimitiveType::Tetrahedron);
            tcd.ear_tet_1 = EarTet{m_mesh.id_tet(ear1), m_mesh.id_face(ear1)};
        } else {
            tcd.ear_tet_1 = EarTet{-1, m_mesh.id_face(ear1)};
        }

        Tuple ear2 = m_mesh.switch_face(m_mesh.switch_edge(m_mesh.switch_vertex(tet)));
        if (!m_mesh.is_boundary_face(ear2)) {
            ear2 = m_mesh.switch_tuple(ear2, PrimitiveType::Tetrahedron);
            tcd.ear_tet_2 = EarTet{m_mesh.id_tet(ear2), m_mesh.id_face(ear2)};
        } else {
            tcd.ear_tet_2 = EarTet{-1, m_mesh.id_face(ear2)};
        }

        tcd.v1 = m_mesh.id_vertex(tet);
        tcd.v2 = m_mesh.id_vertex(m_mesh.switch_vertex(tet));
        tcd.v3 =
            m_mesh.id_vertex(m_mesh.switch_vertex(m_mesh.switch_edge(m_mesh.switch_face(tet))));
        tcd.v4 = m_mesh.id_vertex(m_mesh.switch_vertex(m_mesh.switch_edge(tet)));

        tcd.e12 = m_mesh.id_edge(tet);
        tcd.e14 = m_mesh.id_edge(m_mesh.switch_edge(tet));
        tcd.e24 = m_mesh.id_edge(m_mesh.switch_edge(m_mesh.switch_vertex(tet)));
        tcd.e13 = m_mesh.id_edge(m_mesh.switch_edge(m_mesh.switch_face(tet)));
        tcd.e23 = m_mesh.id_edge(m_mesh.switch_edge(m_mesh.switch_face(m_mesh.switch_vertex(tet))));
        tcd.e34 = m_mesh.id_edge(
            m_mesh.switch_edge(m_mesh.switch_vertex(m_mesh.switch_face(m_mesh.switch_edge(tet)))));

        tet_collapse_data.emplace_back(tcd);
    }

    // local ids for return tuple
    long return_local_vid = -1;
    long return_local_eid = -1;
    long return_local_fid = -1;
    long return_tid = -1;

    // update connectivity for ears
    for (TetCollapseData& data : tet_collapse_data) {
        // prepare all indices
        const long v1 = data.v1;
        const long v2 = data.v2;
        const long v3 = data.v3;
        const long v4 = data.v4;
        const long e12 = data.e12;
        const long e13 = data.e13;
        const long e23 = data.e23;
        const long e14 = data.e14;
        const long e24 = data.e24;
        const long e34 = data.e34;
        const long f_ear_1 = data.ear_tet_1.fid;
        const long f_ear_2 = data.ear_tet_2.fid;
        const long t_ear_1 = data.ear_tet_1.tid;
        const long t_ear_2 = data.ear_tet_2.tid;
        const long t_old = data.tid_old;


        // check by link condition
        assert(t_ear_1 > -1 || t_ear_2 > -1);

        // return_flag == true means this is the tet for the tuple to return
        bool return_flag = false;
        if (t_old == m_operating_tet_id) {
            // found the tet to return
            return_flag = true;
            // prior return tet is ear_2
            return_tid = (t_ear_2 > -1) ? t_ear_2 : t_ear_1;
        }

        // collapse v1 to v2
        // update t_ear_1

        /*
            t_old --> t_ear_2
            f_ear_1 --> f_ear_2
            e13 --> e23
            e14 --> e24
            v1 --> v2 (update later)
        */
        if (t_ear_1 != -1) {
            auto tt = tt_accessor.index_access().vector_attribute(t_ear_1);
            auto tf = tf_accessor.index_access().vector_attribute(t_ear_1);
            auto te = te_accessor.index_access().vector_attribute(t_ear_1);
            auto tv = tv_accessor.index_access().vector_attribute(t_ear_1);

            // get local ids for the return tuple
            if (return_flag && return_tid == t_ear_1) {
                for (int k = 0; k < 4; ++k) {
                    // face
                    if (tf(k) == f_ear_1) {
                        return_local_fid = k;
                    }

                    // vertex
                    if (tv(k) == v1) {
                        return_local_vid = k;
                    }
                }

                for (int k = 0; k < 6; ++k) {
                    // edge
                    if (te(k) == e14) {
                        return_local_eid = k;
                        break;
                    }
                }
            }

            // update
            // face and tet
            for (int k = 0; k < 4; ++k) {
                if (tf(k) == f_ear_1) {
                    assert(tt(k) == t_old);
                    tf(k) = f_ear_2;
                    tt(k) = t_ear_2;
                }
            }

            // edge
            for (int k = 0; k < 6; ++k) {
                if (te(k) == e13) {
                    te(k) = e23;
                }
                if (te(k) == e14) {
                    te(k) = e24;
                }
            }
        }

        // update t_ear_2
        if (t_ear_2 != -1) {
            auto tt = tt_accessor.index_access().vector_attribute(t_ear_2);
            auto tf = tf_accessor.index_access().vector_attribute(t_ear_2);
            auto te = te_accessor.index_access().vector_attribute(t_ear_2);
            auto tv = tv_accessor.index_access().vector_attribute(t_ear_2);


            for (int k = 0; k < 4; ++k) {
                if (tt(k) == t_old) {
                    // assert(tf(k) == f_ear_2);
                    tt(k) = t_ear_1;
                }
            }

            // for return tuple
            if (return_flag && return_tid == t_ear_2) {
                for (int k = 0; k < 4; ++k) {
                    if (tv(k) == v2) {
                        return_local_vid = k;
                    }
                    if (tf(k) == f_ear_2) {
                        return_local_fid = k;
                    }
                }

                for (int k = 0; k < 6; ++k) {
                    if (te(k) == e24) {
                        return_local_eid = k;
                        break;
                    }
                }
            }
        }

        // assign tet for each face
        ft_accessor.index_access().scalar_attribute(f_ear_2) = (t_ear_2 > -1) ? t_ear_2 : t_ear_1;

        data.collapse_new_face_id = f_ear_2;

        // assign tet for each edge
        et_accessor.index_access().scalar_attribute(e23) = (t_ear_2 > -1) ? t_ear_2 : t_ear_1;
        et_accessor.index_access().scalar_attribute(e24) = (t_ear_2 > -1) ? t_ear_2 : t_ear_1;
        et_accessor.index_access().scalar_attribute(e34) = (t_ear_2 > -1) ? t_ear_2 : t_ear_1;

        // assign tet for each vertex
        vt_accessor.index_access().scalar_attribute(v2) = (t_ear_2 > -1) ? t_ear_2 : t_ear_1;
        vt_accessor.index_access().scalar_attribute(v3) = (t_ear_2 > -1) ? t_ear_2 : t_ear_1;
        vt_accessor.index_access().scalar_attribute(v4) = (t_ear_2 > -1) ? t_ear_2 : t_ear_1;
    }

    // update v1 one ring tv
    const long v1 = m_spine_vids[0];
    const long v2 = m_spine_vids[1];

    for (const simplex::Simplex& t : v1_star.simplex_vector(PrimitiveType::Tetrahedron)) {
        const long tid = m_mesh.id(t);
        auto tv = tv_accessor.index_access().vector_attribute(tid);
        for (int i = 0; i < 4; ++i) {
            if (tv(i) == v1) {
                tv(i) = v2;
                break;
            }
        }
    }


    update_cell_hash();
    delete_simplices();

    // debug code
    assert(m_mesh.is_connectivity_valid());

    // std::cout << "return_tid: " << return_tid << std::endl;
    // std::cout << "return_local_vid: " << return_local_vid << std::endl;
    // std::cout << "return_local_eid: " << return_local_eid << std::endl;
    // std::cout << "return_local_fid: " << return_local_fid << std::endl;
    assert(return_tid > -1);
    assert(return_local_fid > -1);
    assert(return_local_eid > -1);
    assert(return_local_vid > -1);
    const long return_tet_hash = hash_accessor.index_access().scalar_attribute(return_tid);


    m_output_tuple =
        Tuple(return_local_vid, return_local_eid, return_local_fid, return_tid, return_tet_hash);
}

std::vector<long> TetMesh::TetMeshOperationExecutor::request_simplex_indices(
    const PrimitiveType type,
    long count)
{
    m_mesh.reserve_attributes(type, m_mesh.capacity(type) + count);

    return m_mesh.request_simplex_indices(type, count);
}


} // namespace wmtk
