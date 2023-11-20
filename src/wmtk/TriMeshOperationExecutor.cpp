
#include "TriMeshOperationExecutor.hpp"
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "SimplicialComplex.hpp"
namespace wmtk {

auto TriMesh::TriMeshOperationExecutor::get_incident_face_data(Tuple t) -> IncidentFaceData
{
    /*         / \
    //  ear1  /   \  ear2
    //       /     \
    //      /       \
    //     X----t----
    */

    // make sure that edge and vertex of the tuple are the same
    for (int i = 0; i < 3; ++i) {
        if (simplex::utils::SimplexComparisons::equal(m_mesh,Simplex::edge(t), Simplex::edge(m_operating_tuple))) {
            break;
        }
        t = m_mesh.next_edge(t);
    }
    assert(simplex::utils::SimplexComparisons::equal(m_mesh,Simplex::edge(t), Simplex::edge(m_operating_tuple)));

    if (!simplex::utils::SimplexComparisons::equal(m_mesh,Simplex::vertex(t), Simplex::vertex(m_operating_tuple))) {
        t = m_mesh.switch_vertex(t);
    }
    assert(simplex::utils::SimplexComparisons::equal(m_mesh,Simplex::vertex(t), Simplex::vertex(m_operating_tuple)));

    const std::array<Tuple, 2> ear_edges{
        {m_mesh.switch_edge(t), m_mesh.switch_edge(m_mesh.switch_vertex(t))}};

    IncidentFaceData face_data;
    face_data.local_operating_tuple = t;
    face_data.fid = m_mesh.id_face(t);
    face_data.opposite_vid = m_mesh.id_vertex(m_mesh.switch_vertex(ear_edges[0]));

    std::transform(
        ear_edges.begin(),
        ear_edges.end(),
        face_data.ears.begin(),
        [&](const Tuple& edge) {
            // accessing ear face id through FF to make it work also at boundaries
            const long ear_fid = ff_accessor.vector_attribute(edge)[edge.m_local_eid];

            return EarData{
                /*.fid = */ ear_fid,
                /*.eid = */ m_mesh.id_edge(edge)};
        });

    return face_data;
}

// constructor
TriMesh::TriMeshOperationExecutor::TriMeshOperationExecutor(
    TriMesh& m,
    const Tuple& operating_tuple,
    Accessor<long>& hash_acc)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge), m.get_flag_accessor(PrimitiveType::Face)}}
    , ff_accessor(m.create_accessor<long>(m.m_ff_handle))
    , fe_accessor(m.create_accessor<long>(m.m_fe_handle))
    , fv_accessor(m.create_accessor<long>(m.m_fv_handle))
    , vf_accessor(m.create_accessor<long>(m.m_vf_handle))
    , ef_accessor(m.create_accessor<long>(m.m_ef_handle))
    , hash_accessor(hash_acc)
    , m_mesh(m)

{
    m_operating_tuple = operating_tuple;
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

    assert(m_incident_face_datas.size() <= 2);
    if (m_incident_face_datas[0].fid != m.id_face(m_operating_tuple)) {
        assert(m_incident_face_datas.size() == 2);
        std::swap(m_incident_face_datas[0], m_incident_face_datas[1]);
    }

    // update hash on all faces in the two-ring neighborhood
    SimplicialComplex hash_update_region(m);
    for (const Simplex& v : edge_closed_star.get_vertices()) {
        const SimplicialComplex v_closed_star = SimplicialComplex::closed_star(m_mesh, v);
        hash_update_region.unify_with_complex(v_closed_star);
    }

    global_simplex_ids_with_potentially_modified_hashes.resize(3);
    for (const Simplex& f : hash_update_region.get_faces()) {
        cell_ids_to_update_hash.push_back(m_mesh.id(f));

        auto faces = wmtk::simplex::faces(m, f, false);
        faces.add(f);
        faces.sort_and_clean();
        auto load = [&](PrimitiveType pt, size_t index) {
            auto simps = faces.simplex_vector(pt);
            std::transform(
                simps.begin(),
                simps.end(),
                std::back_inserter(global_simplex_ids_with_potentially_modified_hashes.at(index)),
                [&](const Simplex& s) {
                    return std::make_tuple(
                        m_mesh.id(s),
                        wmtk::simplex::top_dimension_cofaces_tuples(m_mesh, s));
                });
        };
        load(PrimitiveType::Vertex, 0);
        load(PrimitiveType::Edge, 1);
        load(PrimitiveType::Face, 2);
    }
};

void TriMesh::TriMeshOperationExecutor::delete_simplices()
{
    for (size_t d = 0; d < simplex_ids_to_delete.size(); ++d) {
        for (const long id : simplex_ids_to_delete[d]) {
            flag_accessors[d].index_access().scalar_attribute(id) = 0;
        }
    }
}

void TriMesh::TriMeshOperationExecutor::update_cell_hash()
{
    m_mesh.update_cell_hashes(cell_ids_to_update_hash, hash_accessor);
}

const std::array<std::vector<long>, 3>
TriMesh::TriMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const TriMesh& m)
{
    const SimplicialComplex sc = SimplicialComplex::open_star(m, Simplex::edge(tuple));
    std::array<std::vector<long>, 3> ids;
    for (const Simplex& s : sc.get_simplices()) {
        ids[get_primitive_type_id(s.primitive_type())].emplace_back(m.id(s));
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
        ids[get_primitive_type_id(s.primitive_type())].emplace_back(m.id(s));
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
    const EarData& ear,
    const long new_fid,
    const long old_fid)
{
    //         /|
    //   ear  / |
    //       / new_face
    //      /   |
    //      -----
    if (ear.fid < 0) return;

    auto ear_ff = ff_accessor.index_access().vector_attribute(ear.fid);
    auto ear_fe = fe_accessor.index_access().vector_attribute(ear.fid);
    for (int i = 0; i < 3; ++i) {
        if (ear_ff[i] == old_fid) {
            ear_ff[i] = new_fid;
            ear_fe[i] = ear.eid;
            break;
        }
    }

    ef_accessor.index_access().scalar_attribute(ear.eid) = ear.fid;
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
        const EarData& ear0 = face_data.ears[0];
        const EarData& ear1 = face_data.ears[1];
        const long& f_old = face_data.fid;
        const long& v1 = m_spine_vids[1];

        // TODO: should be detected by link condition
        assert(ear0.fid > -1 || ear1.fid > -1);
        // check manifoldness
        assert(ear0.fid != ear1.fid);

        // change face for v2
        long& new_opp_vf = vf_accessor.index_access().scalar_attribute(face_data.opposite_vid);
        // use ef0 if it exists
        new_opp_vf = (ear0.fid < 0) ? ear1.fid : ear0.fid;

        ef_accessor.index_access().scalar_attribute(ear1.eid) = new_opp_vf;
        vf_accessor.index_access().scalar_attribute(v1) = new_opp_vf;


        EarData new_f0_ear{ear0.fid, ear1.eid};
        // change FF and FE for ears
        update_ids_in_ear(new_f0_ear, ear1.fid, f_old);
        update_ids_in_ear(ear1, ear0.fid, f_old);
    }
};

void TriMesh::TriMeshOperationExecutor::connect_faces_across_spine()
{
    // find the local eid of the spine of the two side of faces
    assert(m_incident_face_datas.size() == 2);
    const long f_old_top = m_incident_face_datas[0].fid;
    const long f0_top = m_incident_face_datas[0].split_f[0];
    const long f1_top = m_incident_face_datas[0].split_f[1];
    const long f_old_bottom = m_incident_face_datas[1].fid;
    const long f0_bottom = m_incident_face_datas[1].split_f[0];
    const long f1_bottom = m_incident_face_datas[1].split_f[1];
    auto ff_old_top = ff_accessor.index_access().vector_attribute(f_old_top);
    auto ff_old_bottom = ff_accessor.index_access().vector_attribute(f_old_bottom);
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
    ff_accessor.index_access().vector_attribute(f0_top)[local_eid_top] = f0_bottom;
    ff_accessor.index_access().vector_attribute(f0_bottom)[local_eid_bottom] = f0_top;
    ff_accessor.index_access().vector_attribute(f1_top)[local_eid_top] = f1_bottom;
    ff_accessor.index_access().vector_attribute(f1_bottom)[local_eid_bottom] = f1_top;
}

void TriMesh::TriMeshOperationExecutor::replace_incident_face(IncidentFaceData& face_data)
{
    // create new faces
    std::vector<long> new_fids = this->request_simplex_indices(PrimitiveType::Face, 2);
    assert(new_fids.size() == 2);

    std::copy(new_fids.begin(), new_fids.end(), face_data.split_f.begin());

    std::vector<long> splitting_edges = this->request_simplex_indices(PrimitiveType::Edge, 1);
    assert(splitting_edges[0] > -1); // TODO: is this assert reasonable at all?
    split_edge_eid = splitting_edges[0];

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

    const long v_opp = face_data.opposite_vid; // opposite vertex
    const long f_old = face_data.fid; // old face
    auto old_fv = fv_accessor.index_access().const_vector_attribute(f_old).eval();
    auto old_fe = fe_accessor.index_access().const_vector_attribute(f_old).eval();
    auto old_ff = ff_accessor.index_access().const_vector_attribute(f_old).eval();

    // f0
    for (int j = 0; j < 2; ++j) {
        int other_index = 1 - j;
        const long f = face_data.split_f[j]; // new face to insert
        const auto& ear = face_data.ears[j];

        const auto& other_ear = face_data.ears[other_index];

        const long other_f = face_data.split_f[other_index];
        const long other_v =
            m_spine_vids[other_index]; // v from the original spine that needs to be replaced
        const long se = split_spine_eids[j]; // new spine edge to replace


        update_ids_in_ear(ear, f, f_old);

        auto fv = fv_accessor.index_access().vector_attribute(f);
        auto fe = fe_accessor.index_access().vector_attribute(f);
        auto ff = ff_accessor.index_access().vector_attribute(f);
        fv = old_fv;
        fe = old_fe;
        ff = old_ff;
        // correct old connectivity
        for (size_t i = 0; i < 3; ++i) {
            // if the original face edge was the other ear's edge then we replace it with thee
            // spline
            if (fe[i] == other_ear.eid) {
                ff[i] = other_f;
                fe[i] = split_edge_eid;
            }

            // replace the input edge iwth the new edge for this triangle
            if (fe[i] == m_operating_edge_id) {
                fe[i] = se;
            }

            // if i find the other vertex then i set it to be the new vertex
            if (fv[i] == other_v) {
                fv[i] = split_new_vid;
            }
        }
        // assign each edge one face
        ef_accessor.index_access().scalar_attribute(ear.eid) = f;
        ef_accessor.index_access().scalar_attribute(se) = f;
        // assign each vertex one face
        vf_accessor.index_access().scalar_attribute(m_spine_vids[j]) = f;
    }

    vf_accessor.index_access().scalar_attribute(v_opp) = new_fids[0];
    vf_accessor.index_access().scalar_attribute(split_new_vid) = new_fids[0];

    ef_accessor.index_access().scalar_attribute(split_edge_eid) = new_fids[0];
    vf_accessor.index_access().scalar_attribute(split_new_vid) = new_fids[0];

    // face neighbors on the other side of the spine are updated separately

    return;
}

void TriMesh::TriMeshOperationExecutor::split_edge()
{
    split_edge_single_mesh();
}

void TriMesh::TriMeshOperationExecutor::split_edge_single_mesh()
{
    simplex_ids_to_delete = get_split_simplices_to_delete(m_operating_tuple, m_mesh);

    // create new vertex (center)
    std::vector<long> new_vids = this->request_simplex_indices(PrimitiveType::Vertex, 1);
    assert(new_vids.size() == 1);
    split_new_vid = new_vids[0];

    // create new edges (spine)
    std::vector<long> new_eids = this->request_simplex_indices(PrimitiveType::Edge, 2);
    assert(new_eids.size() == 2);

    std::copy(new_eids.begin(), new_eids.end(), split_spine_eids.begin());

    for (IncidentFaceData& face_data : m_incident_face_datas) {
        replace_incident_face(face_data);
    }
    assert(m_incident_face_datas.size() <= 2);
    if (m_incident_face_datas.size() > 1) {
        connect_faces_across_spine();
    }

    update_cell_hash();
    delete_simplices();
    // return Tuple new_fid, new_vid that points
    const long new_tuple_fid = m_incident_face_datas[0].split_f[1];
    Tuple& ret = m_output_tuple = m_mesh.edge_tuple_from_id(split_spine_eids[1]);
    if (m_mesh.id_vertex(ret) != split_new_vid) {
        ret = m_mesh.switch_vertex(ret);
        assert(m_mesh.id_vertex(ret) == split_new_vid);
    }
    if (m_mesh.id_face(ret) != new_tuple_fid) {
        ret = m_mesh.switch_face(ret);
        assert(m_mesh.id_face(ret) == new_tuple_fid);
    }
    assert(m_mesh.is_valid(ret, hash_accessor));
}


void TriMesh::TriMeshOperationExecutor::collapse_edge()
{
    collapse_edge_single_mesh();
}


void TriMesh::TriMeshOperationExecutor::collapse_edge_single_mesh()
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

    Tuple& ret = m_output_tuple = m_mesh.edge_tuple_from_id(ret_eid);
    if (m_mesh.id_vertex(ret) != ret_vid) {
        ret = m_mesh.switch_vertex(ret);
    }
    assert(m_mesh.id_vertex(ret) == ret_vid);
    if (m_mesh.id_face(ret) != new_tuple_fid) {
        ret = m_mesh.switch_face(ret);
    }
    assert(m_mesh.id_face(ret) == new_tuple_fid);
    assert(m_mesh.is_valid(ret, hash_accessor));


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
