#include "TetMeshOperationExecutor.hpp"
#include <wmtk/operations/internal/SplitAlternateFacetData.hpp>
#include <wmtk/simplex/IdSimplexCollection.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/half_closed_star_iterable.hpp>
#include <wmtk/simplex/link_single_dimension_iterable.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/top_dimension_cofaces_iterable.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {
namespace {
constexpr static PrimitiveType PE = PrimitiveType::Edge;
constexpr static PrimitiveType PF = PrimitiveType::Triangle;
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


// constructor
TetMesh::TetMeshOperationExecutor::TetMeshOperationExecutor(
    TetMesh& m,
    const Tuple& operating_tuple)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge), m.get_flag_accessor(PrimitiveType::Triangle), m.get_flag_accessor(PrimitiveType::Tetrahedron)}}
    , tt_accessor(*m.m_tt_accessor)
    , tf_accessor(*m.m_tf_accessor)
    , te_accessor(*m.m_te_accessor)
    , tv_accessor(*m.m_tv_accessor)
    , vt_accessor(*m.m_vt_accessor)
    , et_accessor(*m.m_et_accessor)
    , ft_accessor(*m.m_ft_accessor)
    , m_mesh(m)
{
    m_operating_tuple = operating_tuple;
    // store ids of edge and incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(m_mesh.switch_vertex(m_operating_tuple));
    m_operating_face_id = m_mesh.id_face(m_operating_tuple);
    m_operating_tet_id = m_mesh.id_tet(m_operating_tuple);

    if (m_mesh.has_child_mesh()) {
        // get the closed star of the edge
        simplex::SimplexCollection edge_closed_star_vertices(m_mesh);
        const simplex::Simplex edge_operating(m_mesh, PrimitiveType::Edge, operating_tuple);
        for (const Tuple& t : simplex::link_single_dimension_iterable(
                 m_mesh,
                 edge_operating,
                 PrimitiveType::Vertex)) {
            edge_closed_star_vertices.add(PrimitiveType::Vertex, t);
        }
        simplex::faces_single_dimension(
            edge_closed_star_vertices,
            edge_operating,
            PrimitiveType::Vertex);


        assert(
            edge_closed_star_vertices.simplex_vector().size() ==
            edge_closed_star_vertices.simplex_vector(PrimitiveType::Vertex).size());

        // update hash on all tets in the two-ring neighborhood
        simplex::IdSimplexCollection hash_update_region(m);
        for (const simplex::Simplex& v : edge_closed_star_vertices.simplex_vector()) {
            // simplex::top_dimension_cofaces(v, hash_update_region, false);
            for (const Tuple& t : simplex::top_dimension_cofaces_iterable(m_mesh, v)) {
                hash_update_region.add(PrimitiveType::Tetrahedron, t);
            }
        }
        hash_update_region.sort_and_clean();

        global_ids_to_potential_tuples.resize(4);
        simplex::IdSimplexCollection faces(m_mesh);
        faces.reserve(hash_update_region.simplex_vector().size() * 15);

        for (const simplex::IdSimplex& t : hash_update_region.simplex_vector()) {
            faces.add(t);
            const simplex::Simplex s = m.get_simplex(t);
            // faces.add(wmtk::simplex::faces(m, s, false));
            for (const simplex::Simplex& f : simplex::faces(m, s, false)) {
                faces.add(m.get_id_simplex(f));
            }
        }

        // hack (I guess, because the hack below only makes sense with the next line)
        // faces.add(simplex::Simplex(PrimitiveType::Tetrahedron, operating_tuple));

        faces.sort_and_clean();

        for (const simplex::IdSimplex& s : faces) {
            // hack
            // if (s.primitive_type() == PrimitiveType::Tetrahedron) continue;

            const int64_t index = static_cast<int64_t>(s.primitive_type());
            if (!m.has_child_mesh_in_dimension(index)) {
                continue;
            }
            global_ids_to_potential_tuples.at(index).emplace_back(
                m_mesh.id(s),
                wmtk::simplex::top_dimension_cofaces_tuples(m_mesh, m_mesh.get_simplex(s)));
        }

        global_ids_to_potential_tuples.at(3).emplace_back(
            m_mesh.id(simplex::Simplex(m, PrimitiveType::Tetrahedron, operating_tuple)),
            wmtk::simplex::top_dimension_cofaces_tuples(
                m_mesh,
                simplex::Simplex(m, PrimitiveType::Tetrahedron, operating_tuple)));
    }
}

void TetMesh::TetMeshOperationExecutor::delete_simplices()
{
    for (size_t d = 0; d < simplex_ids_to_delete.size(); ++d) {
        for (const int64_t id : simplex_ids_to_delete[d]) {
            flag_accessors[d].index_access().deactivate(id);
        }
    }
}

void TetMesh::TetMeshOperationExecutor::update_cell_hash() {}

const std::array<std::vector<int64_t>, 4>
TetMesh::TetMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const TetMesh& m)
{
    const simplex::SimplexCollection sc = simplex::open_star(m, simplex::Simplex::edge(m, tuple));
    std::array<std::vector<int64_t>, 4> ids;
    for (const simplex::Simplex& s : sc) {
        ids[get_primitive_type_id(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

const std::array<std::vector<int64_t>, 4>
TetMesh::TetMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const TetMesh& m)
{
    std::array<std::vector<int64_t>, 4> ids;
    for (const simplex::IdSimplex& s : simplex::half_closed_star_iterable(m, tuple)) {
        ids[get_primitive_type_id(s.primitive_type())].emplace_back(m.id(s));
    }
    return ids;
}

void TetMesh::TetMeshOperationExecutor::update_ear_connectivity(
    const int64_t ear_tid,
    const int64_t new_tid,
    const int64_t old_tid,
    const int64_t common_fid)
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
    set_split();
    simplex_ids_to_delete = get_split_simplices_to_delete(m_operating_tuple, m_mesh);


    // create new vertex (center)
    std::vector<int64_t> new_vids = this->request_simplex_indices(PrimitiveType::Vertex, 1);
    assert(new_vids.size() == 1);
    const int64_t v_new = new_vids[0];
    m_split_new_vid = v_new;

    // create new edges (spine)
    std::vector<int64_t> new_eids = this->request_simplex_indices(PrimitiveType::Edge, 2);
    assert(new_eids.size() == 2);
    std::copy(new_eids.begin(), new_eids.end(), m_split_new_spine_eids.begin());

    // get incident tets and faces(two cases: loop and boundary)
    // auto incident_tets_and_faces = get_incident_tets_and_faces(m_operating_tuple);
    // const auto& incident_tets = incident_tets_and_faces[0];
    // const auto& incident_faces = incident_tets_and_faces[1];

    const auto [incident_tets, incident_faces] = get_incident_tets_and_faces(m_operating_tuple);
    const bool loop_flag = (incident_tets.size() == incident_faces.size());

    const size_t facet_size = incident_tets.size();
    std::vector<int64_t> new_facet_ids =
        this->request_simplex_indices(PrimitiveType::Tetrahedron, 2 * facet_size);
    assert(new_facet_ids.size() == 2 * facet_size);
    const size_t face_size = incident_faces.size();
    std::vector<int64_t> new_face_ids =
        this->request_simplex_indices(PrimitiveType::Triangle, 2 * face_size);
    assert(new_face_ids.size() == 2 * face_size);

    // create new faces and edges
    std::vector<FaceSplitData> new_incident_face_data;
    for (int64_t i = 0; i < incident_faces.size(); ++i) {
        std::vector<int64_t> splitting_eids = this->request_simplex_indices(PrimitiveType::Edge, 1);

        FaceSplitData fsd;
        fsd.fid_old = m_mesh.id_face(incident_faces[i]);
        std::copy(
            new_face_ids.begin() + 2 * i,
            new_face_ids.begin() + 2 * (i + 1),
            fsd.fid_new.begin());
        fsd.eid_spine_old = m_operating_edge_id;
        fsd.eid_spine_new[0] = new_eids[0]; // redundant
        fsd.eid_spine_new[1] = new_eids[1]; // redundant
        fsd.eid_rib = splitting_eids[0]; // redundant
        fsd.local_operating_tuple = incident_faces[i];
        new_incident_face_data.emplace_back(fsd);
    }


    int64_t incident_face_cnt = new_incident_face_data.size();

    // create new tets
    m_incident_tet_datas.clear();
    for (int64_t i = 0; i < incident_tets.size(); ++i) {
        std::vector<int64_t> split_fids = this->request_simplex_indices(PrimitiveType::Triangle, 1);

        IncidentTetData tsd;
        tsd.local_operating_tuple = incident_tets[i];
        tsd.tid = m_mesh.id_tet(incident_tets[i]);
        std::copy(
            new_facet_ids.begin() + 2 * i,
            new_facet_ids.begin() + 2 * (i + 1),
            tsd.split_t.begin());
        tsd.rib_f = split_fids[0];
        tsd.new_face_id = split_fids[0];

        split_facet_data().add_facet(m_mesh, m_operating_tuple, tsd.split_t);

        // get ears here
        Tuple ear1 = m_mesh.switch_face(m_mesh.switch_edge(incident_tets[i]));
        if (!m_mesh.is_boundary_face(ear1)) {
            ear1 = m_mesh.switch_tuple(ear1, PrimitiveType::Tetrahedron);
            tsd.ears[0] = EarTet{m_mesh.id_tet(ear1), m_mesh.id_face(ear1)};
        } else {
            tsd.ears[0] = EarTet{-1, m_mesh.id_face(ear1)};
        }

        Tuple ear2 = m_mesh.switch_face(m_mesh.switch_edge(m_mesh.switch_vertex(incident_tets[i])));
        if (!m_mesh.is_boundary_face(ear2)) {
            ear2 = m_mesh.switch_tuple(ear2, PrimitiveType::Tetrahedron);
            tsd.ears[1] = EarTet{m_mesh.id_tet(ear2), m_mesh.id_face(ear2)};
        } else {
            tsd.ears[1] = EarTet{-1, m_mesh.id_face(ear2)};
        }

        tsd.new_face_data[0] =
            new_incident_face_data[(i + incident_face_cnt - 1) % incident_face_cnt];
        tsd.new_face_data[1] = new_incident_face_data[i];

        // for multimesh update
        // get the corresponding face data index
        // TODO: add this also to collapse, maybe?
        tsd.incident_face_data_idx[0] = (i + incident_face_cnt - 1) % incident_face_cnt;
        tsd.incident_face_data_idx[1] = i;

        tsd.v0 = m_mesh.id_vertex(incident_tets[i]); // redundant
        tsd.v1 = m_mesh.id_vertex(m_mesh.switch_vertex(incident_tets[i])); // redundant
        tsd.v2 = m_mesh.id_vertex(m_mesh.switch_vertex(
            m_mesh.switch_edge(m_mesh.switch_face(incident_tets[i])))); // put in face
        tsd.v3 = m_mesh.id_vertex(
            m_mesh.switch_vertex(m_mesh.switch_edge(incident_tets[i]))); // put in face rename

        tsd.e01 = m_mesh.id_edge(incident_tets[i]); // redundant
        tsd.e03 = m_mesh.id_edge(m_mesh.switch_edge(incident_tets[i])); // face 1 ear 1 edge
        tsd.e13 = m_mesh.id_edge(
            m_mesh.switch_edge(m_mesh.switch_vertex(incident_tets[i]))); // face 2 ear 2 edge
        tsd.e02 = m_mesh.id_edge(
            m_mesh.switch_edge(m_mesh.switch_face(incident_tets[i]))); // face 1 ear 1 edge
        tsd.e12 = m_mesh.id_edge(m_mesh.switch_edge(
            m_mesh.switch_face(m_mesh.switch_vertex(incident_tets[i])))); // face 2 ear 1 edge
        tsd.e23 = m_mesh.id_edge(m_mesh.switch_edge(m_mesh.switch_vertex(
            m_mesh.switch_face(m_mesh.switch_edge(incident_tets[i]))))); // opposite edge

        m_incident_tet_datas.emplace_back(tsd);
    }

    // incident face data for multimesh and attribute update
    m_incident_face_datas.clear();
    for (int64_t i = 0; i < m_incident_tet_datas.size(); ++i) {
        auto& data = m_incident_face_datas.emplace_back();
        data.fid = m_incident_tet_datas[i].new_face_data[1].fid_old;
        data.ear_eids[0] = m_incident_tet_datas[i].e03;
        data.ear_eids[1] = m_incident_tet_datas[i].e13;
        data.new_edge_id = m_incident_tet_datas[i].new_face_data[1].eid_rib;
        data.split_f[0] = m_incident_tet_datas[i].new_face_data[1].fid_new[0];
        data.split_f[1] = m_incident_tet_datas[i].new_face_data[1].fid_new[1];
        data.local_operating_tuple = m_incident_tet_datas[i].new_face_data[1].local_operating_tuple;
    }

    if (!loop_flag) {
        auto& data = m_incident_face_datas.emplace_back();
        data.fid = m_incident_tet_datas[0].new_face_data[0].fid_old;
        data.ear_eids[0] = m_incident_tet_datas[0].e02;
        data.ear_eids[1] = m_incident_tet_datas[0].e12;
        data.new_edge_id = m_incident_tet_datas[0].new_face_data[0].eid_rib;
        data.split_f[0] = m_incident_tet_datas[0].new_face_data[0].fid_new[0];
        data.split_f[1] = m_incident_tet_datas[0].new_face_data[0].fid_new[1];
        data.local_operating_tuple = m_incident_tet_datas[0].new_face_data[0].local_operating_tuple;
    }

    assert(m_incident_face_datas.size() == new_incident_face_data.size());

// debug code
#ifndef NDEBUG
    for (int64_t i = 0; i < m_incident_face_datas.size(); ++i) {
        assert(m_incident_face_datas[i].fid == new_incident_face_data[i].fid_old);
    }
#endif


    // local ids for return tuple
    int64_t return_local_vid = -1;
    int64_t return_local_eid = -1;
    int64_t return_local_fid = -1;
    int64_t return_tid = -1;

    // these are used only for assertions
#ifndef NDEBUG
    int64_t return_fid = -1;
    int64_t return_split_fid = -1;
#endif

    // update connectivity
    for (int64_t i = 0; i < m_incident_tet_datas.size(); ++i) {
        // prepare all indices
        const auto& data = m_incident_tet_datas[i];
        const int64_t vid_new = v_new;
        const int64_t v0 = data.v0; // m_operating_tuple.vid
        const int64_t v1 = data.v1; // switch_vertex(m_operating_tuple)
        const int64_t v2 = data.v2; // f_old_1 opposite v
        const int64_t v3 = data.v3; // f_old_2 opposite v
        const int64_t e_spine_1 = new_eids[0];
        const int64_t e_spine_2 = new_eids[1];
        const int64_t e_rib_1 = data.new_face_data[0].eid_rib;
        const int64_t e_rib_2 = data.new_face_data[1].eid_rib;
        const int64_t e01 = data.e01;
        const int64_t e02 = data.e02;
        const int64_t e12 = data.e12;
        const int64_t e03 = data.e03;
        const int64_t e13 = data.e13;
        const int64_t e23 = data.e23;
        const int64_t f_ear_1 = data.ears[0].fid;
        const int64_t f_ear_2 = data.ears[1].fid;
        const int64_t f1 = data.new_face_data[0].fid_new[0];
        const int64_t f2 = data.new_face_data[0].fid_new[1];
        const int64_t f_old_1 = data.new_face_data[0].fid_old; // f1 + f2
        const int64_t f3 = data.new_face_data[1].fid_new[0];
        const int64_t f4 = data.new_face_data[1].fid_new[1];
        const int64_t f_old_2 = data.new_face_data[1].fid_old; // f3 + f4
        const int64_t f_rib = data.rib_f;
        const int64_t t_ear_1 = data.ears[0].tid;
        const int64_t t_ear_2 = data.ears[1].tid;
        const int64_t t1 = data.split_t[0];
        const int64_t t2 = data.split_t[1];
        const int64_t t_old = data.tid;
        int64_t t_f1; // prev t1
        int64_t t_f2; // prev t2
        int64_t t_f3; // next t1
        int64_t t_f4; // next t2

        // /////////////////////////////////////////////////
        // // debug code , to delete
        // if (e_spine_1 == 2223) {
        //     wmtk::logger().info(
        //         "edge 2223 is created in tet {} face {} edge {} as spine edge 1, belongs to new "
        //         "tet {}, face {} and face {}, loop flag {}, left ear {}, right ear{}, right "
        //         "neighbor{}, left ear face {}, right ear face {}",
        //         t_old,
        //         f_old_1,
        //         e01,
        //         t1,
        //         f1,
        //         f3,
        //         loop_flag,
        //         t_ear_1,
        //         t_ear_2,
        //         t2,
        //         f_ear_1,
        //         f_ear_2);
        // }
        // if (e_spine_2 == 2223) {
        //     wmtk::logger().info(
        //         "edge 2223 is created in tet {} face {} edge {} as spine edge 2, belongs to new "
        //         "tet {}, face {} and face {}, loop flag {}",
        //         t_old,
        //         f_old_2,
        //         e01,
        //         t2,
        //         f2,
        //         f4,
        //         loop_flag);
        // }
        // if (e_rib_1 == 2223) {
        //     wmtk::logger().info(
        //         "edge 2223 is created in tet {} face {} as rib edge 1, belongs to new "
        //         "tet {} and {}, face {} and face {}, loop flag {}",
        //         t_old,
        //         f_old_1,
        //         t1,
        //         t2,
        //         f1,
        //         f3,
        //         loop_flag);
        // }
        // if (e_rib_2 == 2223) {
        //     wmtk::logger().info(
        //         "edge 2223 is created in tet {} face {} as rib edge 2, belongs to new "
        //         "tet {} and {}, face {} and face {}, loop flag {}",
        //         t_old,
        //         f_old_2,
        //         t1,
        //         t2,
        //         f2,
        //         f4,
        //         loop_flag);
        // }

        // /////////////////////////////////////////////////

        // for return tuple
        // return_flag == true means this is the tet for the tuple to return

        bool return_flag = false;
        if (t_old == m_operating_tet_id) {
            return_tid = t2;

            logger().trace("split fid is {}", f_rib);
            logger().trace("fids {} {} are joined by edge {}", f3, f4, e_rib_2);
#ifndef NDEBUG
            return_fid = f4;
            return_split_fid = f_rib;
#endif
            return_flag = true;
        }
        int64_t prev_index = (i - 1 + m_incident_tet_datas.size()) % m_incident_tet_datas.size();
        int64_t next_index = (i + 1 + m_incident_tet_datas.size()) % m_incident_tet_datas.size();

        if (loop_flag) {
            t_f1 = m_incident_tet_datas[prev_index].split_t[0];
            t_f2 = m_incident_tet_datas[prev_index].split_t[1];
            t_f3 = m_incident_tet_datas[next_index].split_t[0];
            t_f4 = m_incident_tet_datas[next_index].split_t[1];
        } else {
            if (m_incident_tet_datas.size() == 1) {
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
                    t_f1 = m_incident_tet_datas[prev_index].split_t[0];
                    t_f2 = m_incident_tet_datas[prev_index].split_t[1];
                }
                if (i == m_incident_tet_datas.size() - 1) {
                    // no next
                    t_f3 = -1;
                    t_f4 = -1;
                } else {
                    t_f3 = m_incident_tet_datas[next_index].split_t[0];
                    t_f4 = m_incident_tet_datas[next_index].split_t[1];
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
                v1 --> v_new
                e13 --> e_split2
                e12 --> e_split1
                e01 --> e_spine1
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
                    if (te(k) == e01) {
                        return_local_eid = k;
                        break;
                    }
                }
            }


            for (size_t k = 0; k < 4; ++k) {
                // vertices
                if (tv(k) == v1) {
                    tv(k) = vid_new;
                }

                // faces and tets
                if (tf(k) == f_old_1) {
                    // local fid for multimesh update
                    m_incident_tet_datas[i].incident_face_local_fid[0] = k;

                    tf(k) = f1;
                    tt(k) = t_f1;
                }
                if (tf(k) == f_old_2) {
                    // local fid for multimesh update
                    m_incident_tet_datas[i].incident_face_local_fid[1] = k;

                    tf(k) = f3;
                    tt(k) = t_f3;
                }
                if (tf(k) == f_ear_2) {
                    tf(k) = f_rib;
                    tt(k) = t2;
                }
            }

            for (size_t k = 0; k < 6; ++k) {
                // edges
                if (te(k) == e13) {
                    te(k) = e_rib_2;
                }
                if (te(k) == e12) {
                    te(k) = e_rib_1;
                }
                if (te(k) == e01) {
                    te(k) = e_spine_1;
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
                v0 --> v_new
                e03 --> e_split2
                e02 --> e_split1
                e01 --> e_spine2
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
                if (tv(k) == v0) {
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
                    tf(k) = f_rib;
                    tt(k) = t1;
                }
            }

            for (size_t k = 0; k < 6; ++k) {
                // edges
                if (te(k) == e03) {
                    te(k) = e_rib_2;
                }
                if (te(k) == e02) {
                    te(k) = e_rib_1;
                }
                if (te(k) == e01) {
                    te(k) = e_spine_2;
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
        ft_accessor.index_access().scalar_attribute(f_rib) = t1;

        // assign each edge one tet
        et_accessor.index_access().scalar_attribute(e02) = t1;
        et_accessor.index_access().scalar_attribute(e12) = t2;
        et_accessor.index_access().scalar_attribute(e03) = t1;
        et_accessor.index_access().scalar_attribute(e13) = t2;
        et_accessor.index_access().scalar_attribute(e23) = t1;
        et_accessor.index_access().scalar_attribute(e_spine_1) = t1;
        et_accessor.index_access().scalar_attribute(e_spine_2) = t2;
        et_accessor.index_access().scalar_attribute(e_rib_1) = t1;
        et_accessor.index_access().scalar_attribute(e_rib_2) = t1;

        // assign each vertex one tet
        vt_accessor.index_access().scalar_attribute(v0) = t1;
        vt_accessor.index_access().scalar_attribute(v1) = t2;
        vt_accessor.index_access().scalar_attribute(v2) = t1;
        vt_accessor.index_access().scalar_attribute(v3) = t1;
        vt_accessor.index_access().scalar_attribute(vid_new) = t1;
    }


    // update hash and delete simplices
    update_cell_hash();
    delete_simplices();


    assert(return_tid > -1);
    assert(return_local_vid > -1);
    assert(return_local_eid > -1);
    assert(return_local_fid > -1);

    assert(return_local_vid == m_operating_tuple.local_vid());
    assert(return_local_eid == m_operating_tuple.local_eid());
    assert(return_local_fid == m_operating_tuple.local_fid());
    m_output_tuple = Tuple(return_local_vid, return_local_eid, return_local_fid, return_tid);

    assert(m_split_new_vid == m_mesh.id(simplex::Simplex::vertex(m_mesh, m_output_tuple)));
    assert(m_split_new_spine_eids[1] == m_mesh.id(simplex::Simplex::edge(m_mesh, m_output_tuple)));
    assert(return_fid == m_mesh.id(simplex::Simplex::face(m_mesh, m_output_tuple)));
    assert(return_tid == m_mesh.id(simplex::Simplex::tetrahedron(m_mesh, m_output_tuple)));

    logger().trace(
        "split fid is {}",
        m_mesh.id(simplex::Simplex::face(m_mesh, m_mesh.switch_tuples(m_output_tuple, {PE, PF}))));
    // assert(m_mesh.id(simplex::Simplex::edge(m_mesh.switch_tuples(m_output_tuple, {PE}))) =
    // return_face_spine_eid);
    assert(
        m_mesh.id(simplex::Simplex::face(m_mesh, m_mesh.switch_tuples(m_output_tuple, {PE, PF}))) ==
        return_split_fid);
    assert(!m_mesh.is_boundary_face(m_mesh.switch_tuples(m_output_tuple, {PE, PF})));
}

void TetMesh::TetMeshOperationExecutor::collapse_edge()
{
    set_collapse();
    is_collapse = true;
    simplex_ids_to_delete = get_collapse_simplices_to_delete(m_operating_tuple, m_mesh);

    // collect star before changing connectivity
    // update all tv's after other updates
    simplex::IdSimplexCollection v0_star(m_mesh);
    v0_star.reserve(32);
    for (const Tuple& t : simplex::top_dimension_cofaces_iterable(
             m_mesh,
             simplex::Simplex::vertex(m_mesh, m_operating_tuple))) {
        v0_star.add(PrimitiveType::Tetrahedron, t);
    }


    // collect incident tets and their ears
    // loop case and boundary case
    // auto incident_tets_and_faces = get_incident_tets_and_faces(m_operating_tuple);
    // const auto& incident_tets = incident_tets_and_faces[0];

    auto [incident_tets, incident_faces] = get_incident_tets_and_faces(m_operating_tuple);


    for (const Tuple& tet : incident_tets) {
        IncidentTetData tcd;
        tcd.local_operating_tuple = tet;
        tcd.tid = m_mesh.id_tet(tet);

        // get ears
        Tuple ear1 = m_mesh.switch_face(m_mesh.switch_edge(tet));
        if (!m_mesh.is_boundary_face(ear1)) {
            ear1 = m_mesh.switch_tuple(ear1, PrimitiveType::Tetrahedron);
            tcd.ears[0] = EarTet{m_mesh.id_tet(ear1), m_mesh.id_face(ear1)};
        } else {
            tcd.ears[0] = EarTet{-1, m_mesh.id_face(ear1)};
        }

        Tuple ear2 = m_mesh.switch_face(m_mesh.switch_edge(m_mesh.switch_vertex(tet)));
        if (!m_mesh.is_boundary_face(ear2)) {
            ear2 = m_mesh.switch_tuple(ear2, PrimitiveType::Tetrahedron);
            tcd.ears[1] = EarTet{m_mesh.id_tet(ear2), m_mesh.id_face(ear2)};
        } else {
            tcd.ears[1] = EarTet{-1, m_mesh.id_face(ear2)};
        }

        tcd.v0 = m_mesh.id_vertex(tet);
        tcd.v1 = m_mesh.id_vertex(m_mesh.switch_vertex(tet));
        tcd.v2 =
            m_mesh.id_vertex(m_mesh.switch_vertex(m_mesh.switch_edge(m_mesh.switch_face(tet))));
        tcd.v3 = m_mesh.id_vertex(m_mesh.switch_vertex(m_mesh.switch_edge(tet)));

        tcd.e01 = m_mesh.id_edge(tet);
        tcd.e03 = m_mesh.id_edge(m_mesh.switch_edge(tet));
        tcd.e13 = m_mesh.id_edge(m_mesh.switch_edge(m_mesh.switch_vertex(tet)));
        tcd.e02 = m_mesh.id_edge(m_mesh.switch_edge(m_mesh.switch_face(tet)));
        tcd.e12 = m_mesh.id_edge(m_mesh.switch_edge(m_mesh.switch_face(m_mesh.switch_vertex(tet))));
        tcd.e23 = m_mesh.id_edge(
            m_mesh.switch_edge(m_mesh.switch_vertex(m_mesh.switch_face(m_mesh.switch_edge(tet)))));

        m_incident_tet_datas.emplace_back(tcd);
    }

    // incident face data for multimesh and attribute update
    m_incident_face_datas.clear();
    for (int64_t i = 0; i < m_incident_tet_datas.size(); ++i) {
        auto& data = m_incident_face_datas.emplace_back();
        data.ear_eids[0] = m_incident_tet_datas[i].e03;
        data.ear_eids[1] = m_incident_tet_datas[i].e13;
        data.new_edge_id = data.ear_eids[1];
    }

    if (incident_tets.size() != incident_faces.size()) {
        auto& data = m_incident_face_datas.emplace_back();
        data.ear_eids[0] = m_incident_tet_datas[0].e02;
        data.ear_eids[1] = m_incident_tet_datas[0].e12;
        data.new_edge_id = data.ear_eids[1];
    }

    assert(m_incident_face_datas.size() == incident_faces.size());


    // local ids for return tuple
    int64_t return_local_vid = -1;
    int64_t return_local_eid = -1;
    int64_t return_local_fid = -1;
    int64_t return_tid = -1;

    std::map<int64_t, int64_t> edge_replacement;

    // update connectivity for ears
    for (IncidentTetData& data : m_incident_tet_datas) {
        // prepare all indices
        const int64_t v0 = data.v0;
        const int64_t v1 = data.v1;
        const int64_t v2 = data.v2;
        const int64_t v3 = data.v3;
        const int64_t e01 = data.e01;
        const int64_t e02 = data.e02;
        const int64_t e12 = data.e12;
        const int64_t e03 = data.e03;
        const int64_t e13 = data.e13;
        const int64_t e23 = data.e23;
        const int64_t f_ear_1 = data.ears[0].fid;
        const int64_t f_ear_2 = data.ears[1].fid;
        const int64_t t_ear_1 = data.ears[0].tid;
        const int64_t t_ear_2 = data.ears[1].tid;
        const int64_t t_old = data.tid;

        edge_replacement[e02] = e12;
        edge_replacement[e03] = e13;

        /////////////////////////////////////////////////
        // // debug code
        // if (e13 == 2223) {
        //     wmtk::logger().info(
        //         "edge 2223 in tet {} is assigned to tet {} face {} as merged edge 03->13, "
        //         "replacing edge {}, right ear is tet {} face {} edge {}",
        //         t_old,
        //         t_ear_1,
        //         f_ear_1,
        //         e03,
        //         t_ear_2,
        //         f_ear_2,
        //         e13);
        // }

        // if (e12 == 2223) {
        //     wmtk::logger().info(
        //         "edge 2223 in tet {} is assigned to tet {} face {} as merged edge 02->12, "
        //         "replacing edge {}, right ear is tet {} face {} edge {}",
        //         t_old,
        //         t_ear_1,
        //         f_ear_1,
        //         e02,
        //         t_ear_2,
        //         f_ear_2,
        //         e12);
        // }
        // /////////////////////////////////////////////////


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

        // collapse v0 to v1
        // update t_ear_1

        /*
            t_old --> t_ear_2
            f_ear_1 --> f_ear_2
            e02 --> e12
            e03 --> e13
            v0 --> v1 (update later)
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
                    if (tv(k) == v0) {
                        return_local_vid = k;
                    }
                }

                for (int k = 0; k < 6; ++k) {
                    // edge
                    if (te(k) == e03) {
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
                if (te(k) == e02) {
                    te(k) = e12;
                }
                if (te(k) == e03) {
                    te(k) = e13;
                }
            }
        }

        // update t_ear_2
        if (t_ear_2 != -1) {
            auto tt = tt_accessor.index_access().vector_attribute(t_ear_2);
            auto tf = tf_accessor.index_access().vector_attribute(t_ear_2);
            auto te = te_accessor.index_access().vector_attribute(t_ear_2);
            auto tv = tv_accessor.index_access().vector_attribute(t_ear_2);


            // for return tuple
            if (return_flag && return_tid == t_ear_2) {
                for (int k = 0; k < 4; ++k) {
                    if (tv(k) == v1) {
                        return_local_vid = k;
                    }
                    if (tf(k) == f_ear_2) {
                        return_local_fid = k;
                    }
                }

                for (int k = 0; k < 6; ++k) {
                    if (te(k) == e13) {
                        return_local_eid = k;
                        break;
                    }
                }
            }

            for (int k = 0; k < 4; ++k) {
                if (tt(k) == t_old) {
                    // assert(tf(k) == f_ear_2);
                    tt(k) = t_ear_1;
                }
            }
        }

        const int64_t t_ear_valid = (t_ear_2 > -1) ? t_ear_2 : t_ear_1;
        // for multimesh update

        data.merged_face_tid = t_ear_valid;
        // assign tet for each face
        ft_accessor.index_access().scalar_attribute(f_ear_2) = t_ear_valid;

        data.new_face_id = f_ear_2;

        // assign tet for each edge
        et_accessor.index_access().scalar_attribute(e12) = t_ear_valid;
        et_accessor.index_access().scalar_attribute(e13) = t_ear_valid;
        et_accessor.index_access().scalar_attribute(e23) = t_ear_valid;

        // assign tet for each vertex
        vt_accessor.index_access().scalar_attribute(v1) = t_ear_valid;
        vt_accessor.index_access().scalar_attribute(v2) = t_ear_valid;
        vt_accessor.index_access().scalar_attribute(v3) = t_ear_valid;
    }

    // update v0 one ring tv
    // update ear edge replacements
    const int64_t v0 = m_spine_vids[0];
    const int64_t v1 = m_spine_vids[1];

    for (const simplex::IdSimplex& t : v0_star) {
        // for (const simplex::Simplex& t : v0_star.simplex_vector(PrimitiveType::Tetrahedron)) {
        const int64_t tid = m_mesh.id(t);
        auto tv = tv_accessor.index_access().vector_attribute(tid);
        auto te = te_accessor.index_access().vector_attribute(tid);
        for (int i = 0; i < 4; ++i) {
            if (tv(i) == v0) {
                tv(i) = v1;
                break;
            }
        }
        for (int i = 0; i < 6; ++i) {
            if (edge_replacement.find(te(i)) != edge_replacement.end()) {
                te(i) = edge_replacement[te(i)];
            }
        }
    }


    update_cell_hash();
    delete_simplices();

    // debug code
    assert(m_mesh.is_connectivity_valid());
    assert(return_tid > -1);
    assert(return_local_fid > -1);
    assert(return_local_eid > -1);
    assert(return_local_vid > -1);
    m_output_tuple = Tuple(return_local_vid, return_local_eid, return_local_fid, return_tid);

    assert(m_mesh.id_vertex(m_output_tuple) == v1);
}

std::vector<int64_t> TetMesh::TetMeshOperationExecutor::request_simplex_indices(
    const PrimitiveType type,
    int64_t count)
{
    m_mesh.guarantee_more_attributes(type, count);

    return m_mesh.request_simplex_indices(type, count);
}


} // namespace wmtk
