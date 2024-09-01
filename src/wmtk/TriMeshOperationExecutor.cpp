// kills a gcc-13 warning
#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic push
// this warning only exists for gcc >= 13.0
#if __GNUC__ > 12
#pragma GCC diagnostic ignored "-Wdangling-pointer"
#endif // check gnu version
#endif
// clang-format off
#include <Eigen/Core>

#include <Eigen/src/Core/MapBase.h>
// cland-format on

#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "TriMeshOperationExecutor.hpp"

#include <wmtk/operations/internal/SplitAlternateFacetData.hpp>

namespace wmtk {

    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    Tuple TriMesh::TriMeshOperationExecutor::next_edge(const Tuple& tuple) const { return m_mesh.switch_tuples(tuple,{PV,PE}); }
    Tuple TriMesh::TriMeshOperationExecutor::prev_edge(const Tuple& tuple) const { return m_mesh.switch_tuples(tuple,{PE,PV}); }
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
        if (simplex::utils::SimplexComparisons::equal(
                m_mesh,
                simplex::Simplex::edge(m_mesh, t),
                simplex::Simplex::edge(m_mesh, m_operating_tuple))) {
            break;
        }
        t = next_edge(t);
    }
    assert(simplex::utils::SimplexComparisons::equal(
        m_mesh,
        simplex::Simplex::edge(m_mesh, t),
        simplex::Simplex::edge(m_mesh, m_operating_tuple)));

    if (!simplex::utils::SimplexComparisons::equal(
            m_mesh,
            simplex::Simplex::vertex(m_mesh, t),
            simplex::Simplex::vertex(m_mesh, m_operating_tuple))) {
        t = m_mesh.switch_vertex(t);
    }
    assert(simplex::utils::SimplexComparisons::equal(
        m_mesh,
        simplex::Simplex::vertex(m_mesh, t),
        simplex::Simplex::vertex(m_mesh, m_operating_tuple)));

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
            const int64_t ear_fid = ff_accessor.const_vector_attribute(edge)[edge.m_local_eid];

            return EarData{
                /*.fid = */ ear_fid,
                /*.eid = */ m_mesh.id_edge(edge)};
        });

    return face_data;
}

// constructor
TriMesh::TriMeshOperationExecutor::TriMeshOperationExecutor(
    TriMesh& m,
    const Tuple& operating_tuple)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge), m.get_flag_accessor(PrimitiveType::Triangle)}}
    , ff_accessor(*m.m_ff_accessor)
    , fe_accessor(*m.m_fe_accessor)
    , fv_accessor(*m.m_fv_accessor)
    , vf_accessor(*m.m_vf_accessor)
    , ef_accessor(*m.m_ef_accessor)
    , m_mesh(m)

{
    assert(m.is_connectivity_valid());
    m_operating_tuple = operating_tuple;
    // store ids of edge and incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(m_mesh.switch_vertex(m_operating_tuple));

}

void TriMesh::TriMeshOperationExecutor::delete_simplices()
{
    for (size_t d = 0; d < simplex_ids_to_delete.size(); ++d) {
        wmtk::logger().trace("Deleting {} {}-simplices [{}]", simplex_ids_to_delete[d].size(), d, fmt::join(simplex_ids_to_delete[d],","));
        for (const int64_t id : simplex_ids_to_delete[d]) {
            flag_accessors[d].index_access().scalar_attribute(id) = 0;
        }
    }
}

void TriMesh::TriMeshOperationExecutor::update_cell_hash()
{
}

const std::array<std::vector<int64_t>, 3>
TriMesh::TriMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const TriMesh& m)
{
    const simplex::SimplexCollection sc = simplex::open_star(m, simplex::Simplex::edge(m, tuple));
    std::array<std::vector<int64_t>, 3> ids;
    for (const simplex::Simplex& s : sc) {
        ids[get_primitive_type_id(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

const std::array<std::vector<int64_t>, 3>
TriMesh::TriMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const TriMesh& m)
{
    std::array<std::vector<int64_t>, 3> ids;
    auto get_sc = [&]() -> simplex::SimplexCollection {
    if(m.is_free()) {
        simplex::Simplex simp (m,PrimitiveType::Triangle,tuple);

        simplex::SimplexCollection sc = simplex::faces(m, simp);
        sc.add(simp);
        return sc;
    } else {
    const simplex::SimplexCollection vertex_open_star =
        simplex::open_star(m, simplex::Simplex::vertex(m, tuple));
    const simplex::SimplexCollection edge_closed_star =
        simplex::closed_star(m, simplex::Simplex::edge(m, tuple));

    simplex::SimplexCollection sc=
        simplex::SimplexCollection::get_intersection(vertex_open_star, edge_closed_star);

    return sc;
    }
    };
    for (const simplex::Simplex& s : get_sc()) {
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
    const int64_t new_fid,
    const int64_t old_fid)
{
    //         /|
    //   ear  / |
    //       / new_face
    //      /   |
    //      -----
    if (ear.fid < 0) {
        return;
    }

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

    for (auto& face_data : m_incident_face_datas) {
        const EarData& ear0 = face_data.ears[0];
        const EarData& ear1 = face_data.ears[1];
        const int64_t& f_old = face_data.fid;
        const int64_t& v1 = m_spine_vids[1];

        // TODO: should be detected by link condition
        assert(ear0.fid > -1 || ear1.fid > -1);
        // check manifoldness
        assert(ear0.fid != ear1.fid);

        // change face for v2
        int64_t& new_opp_vf = vf_accessor.index_access().scalar_attribute(face_data.opposite_vid);
        // use ef0 if it exists
        new_opp_vf = (ear0.fid < 0) ? ear1.fid : ear0.fid;

        face_data.new_edge_id = ear1.eid;
        // for multimesh update
        face_data.merged_edge_fid = new_opp_vf;

        int64_t& ef_val = ef_accessor.index_access().scalar_attribute(ear1.eid);
        int64_t& vf_val = vf_accessor.index_access().scalar_attribute(v1);


        ef_val = new_opp_vf;
        vf_val = new_opp_vf;


        EarData new_f0_ear{ear0.fid, ear1.eid};
        // change FF and FE for ears
        update_ids_in_ear(new_f0_ear, ear1.fid, f_old);
        update_ids_in_ear(ear1, ear0.fid, f_old);
    }
}

void TriMesh::TriMeshOperationExecutor::connect_faces_across_spine()
{
    // find the local eid of the spine of the two side of faces
    assert(m_incident_face_datas.size() == 2);
    const int64_t f_old_top = m_incident_face_datas[0].fid;
    const int64_t f0_top = m_incident_face_datas[0].split_f[0];
    const int64_t f1_top = m_incident_face_datas[0].split_f[1];
    const int64_t f_old_bottom = m_incident_face_datas[1].fid;
    const int64_t f0_bottom = m_incident_face_datas[1].split_f[0];
    const int64_t f1_bottom = m_incident_face_datas[1].split_f[1];
    auto ff_old_top = ff_accessor.index_access().const_vector_attribute(f_old_top);
    auto ff_old_bottom = ff_accessor.index_access().const_vector_attribute(f_old_bottom);
    assert(m_mesh.capacity(PrimitiveType::Triangle) > f0_top);
    assert(m_mesh.capacity(PrimitiveType::Triangle) > f1_top);
    assert(m_mesh.capacity(PrimitiveType::Triangle) > f0_bottom);
    assert(m_mesh.capacity(PrimitiveType::Triangle) > f1_bottom);

    // local edge ids are the same for both, f1 and f2
    int64_t local_eid_top = -1;
    int64_t local_eid_bottom = -1;
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
    const auto& new_fids = face_data.split_f;

    if(!m_mesh.is_free()) {
        std::vector<int64_t> splitting_edges = this->request_simplex_indices(PrimitiveType::Edge, 1);
        assert(splitting_edges[0] > -1); // TODO: is this assert reasonable at all?
        int64_t& split_edge_eid = face_data.new_edge_id;
        split_edge_eid = splitting_edges[0];
    }

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

    const int64_t v_opp = face_data.opposite_vid; // opposite vertex
    const int64_t f_old = face_data.fid; // old face
    auto old_fv = fv_accessor.index_access().const_vector_attribute(f_old).eval();
    auto old_fe = fe_accessor.index_access().const_vector_attribute(f_old).eval();
    auto old_ff = ff_accessor.index_access().const_vector_attribute(f_old).eval();

    // f0
    for (int j = 0; j < 2; ++j) {
        int other_index = 1 - j;
        const int64_t f = face_data.split_f[j]; // new face to insert
        const auto& ear = face_data.ears[j];

        const auto& other_ear = face_data.ears[other_index];

        const int64_t other_f = face_data.split_f[other_index];
        const int64_t other_v =
            m_spine_vids[other_index]; // v from the original spine that needs to be replaced
        const int64_t se = split_spine_eids[j]; // new spine edge to replace

        assert(se != -1);

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
                if(m_mesh.is_free()) {
                    fe[i] = m_free_split_e[j];
                } else {
                    const int64_t& split_edge_eid = face_data.new_edge_id;
                    ff[i] = other_f;
                    fe[i] = split_edge_eid;
                    logger().trace("ff[{},{}] = {}", f,i,other_f);
                    logger().trace("fe[{},{}] = {}", f,i,split_edge_eid);
                }
            }

            // replace the input edge iwth the new edge for this triangle
            if (fe[i] == m_operating_edge_id) {
                logger().trace("fe[{},{}] = {}", f,i,se);
                fe[i] = se;
            }

            // if i find the other vertex then i set it to be the new vertex
            if (fv[i] == other_v) {
                if(m_mesh.is_free()) {
                    fv[i] = m_free_split_v[j];
                } else {
                    fv[i] = split_new_vid;
                }
            }
        }
        logger().trace("ef[{}] = {}", ear.eid,f);
        logger().trace("ef[{}] = {}", se,f);
        // assign each edge one face
        ef_accessor.index_access().scalar_attribute(ear.eid) = f;
        ef_accessor.index_access().scalar_attribute(se) = f;
        // assign each vertex one face
        logger().trace("vf[{}] = {}", m_spine_vids[j],f);
        vf_accessor.index_access().scalar_attribute(m_spine_vids[j]) = f;
    }

    vf_accessor.index_access().scalar_attribute(v_opp) = new_fids[0];

    if(m_mesh.is_free()) {
        for(size_t j = 0; j < 2; ++j) {
            ef_accessor.index_access().scalar_attribute(m_free_split_e[j]) = new_fids[j];
            vf_accessor.index_access().scalar_attribute(m_free_split_v[j]) = new_fids[j];
        }

        const int64_t f = face_data.split_f[1]; // new face to insert
        auto fv = fv_accessor.index_access().vector_attribute(f);
        for(int j = 0; j < 3; ++j) {
            if(fv(j) == face_data.opposite_vid) {
                logger().trace("fv[{},{}] = {}", f,j,split_new_vid);
                fv(j) = split_new_vid;

                logger().trace("vf[{}] = {}", split_new_vid, new_fids[1]);
                vf_accessor.index_access().scalar_attribute(split_new_vid) = new_fids[1];
            }
        }

    } else {
        const int64_t& split_edge_eid = face_data.new_edge_id;
        logger().trace("ef[{}] = {}", split_edge_eid, new_fids[0]);
        logger().trace("vf[{}] = {}", split_new_vid, new_fids[0]);
        ef_accessor.index_access().scalar_attribute(split_edge_eid) = new_fids[0];
        vf_accessor.index_access().scalar_attribute(split_new_vid) = new_fids[0];
    }

    // face neighbors on the other side of the spine are updated separately

    return;
}

void TriMesh::TriMeshOperationExecutor::split_edge_precompute()
{
    set_split();
    // need to write:
    // * global_ids_to_potential_tuples
    // * m_incident_face_datas
    // * simplex_ids_to_delete
    simplex_ids_to_delete = get_split_simplices_to_delete(m_operating_tuple, m_mesh);

    std::vector<Tuple> adjacent_facets = wmtk::simplex::top_dimension_cofaces_tuples(m_mesh, simplex::Simplex::edge(m_mesh,m_operating_tuple));

    assert(m_operating_tuple == adjacent_facets[0]);

    assert(m_incident_face_datas.empty());

    std::transform(adjacent_facets.begin(), adjacent_facets.end(), std::back_inserter(m_incident_face_datas),
            [&](const Tuple& t) {
            return get_incident_face_data(t);
            });

    assert(m_incident_face_datas[0].fid == m_mesh.id_face(m_operating_tuple));
    assert(m_incident_face_datas[0].local_operating_tuple == m_operating_tuple);

    std::vector<std::vector<Tuple>> bsc(2);
    if(m_mesh.has_child_mesh_in_dimension(0)) {
        auto& this_bsc = bsc[0];
        this_bsc.reserve(m_incident_face_datas.size() + 2);

        this_bsc.emplace_back(m_operating_tuple);
        this_bsc.emplace_back(m_mesh.switch_vertex(m_operating_tuple));
        for(const auto& data: m_incident_face_datas) {
            this_bsc.emplace_back(m_mesh.switch_tuples(data.local_operating_tuple,
                        {PrimitiveType::Edge,PrimitiveType::Vertex}));
        }
    }
    if(m_mesh.has_child_mesh_in_dimension(1)) {
        auto& this_bsc = bsc[1];
        /*
        this_bsc.reserve(3 * m_incident_face_datas.size());
            for(const auto& facet: m_incident_face_datas) {
                        auto f = wmtk::simplex::faces(m_mesh, f, false);
                        std::copy(f.begin(),f.end(), std::back_inserter(this_bsc));
            }
            std::sort(this_bsc.begin(),this_bsc.end());
            this_bsc.erase(std::unique(this_bsc.begin(),this_bsc.end()),this_bsc.end());
        }
        */
}



    const simplex::SimplexCollection edge_closed_star =
        simplex::closed_star(m_mesh, simplex::Simplex::edge(m_mesh, m_operating_tuple));




    // update hash on all faces in the two-ring neighborhood
    simplex::SimplexCollection hash_update_region(m_mesh);
    for (const simplex::Simplex& v : edge_closed_star.simplex_vector(PrimitiveType::Vertex)) {
        const simplex::SimplexCollection v_closed_star = simplex::top_dimension_cofaces(m_mesh, v);
        hash_update_region.add(v_closed_star);
    }
    hash_update_region.sort_and_clean();

    global_ids_to_potential_tuples.resize(3);
    simplex::SimplexCollection faces(m_mesh);

    for (const simplex::Simplex& f : hash_update_region.simplex_vector(PrimitiveType::Triangle)) {

        faces.add(wmtk::simplex::faces(m_mesh, f, false));
        faces.add(f);
    }

    faces.sort_and_clean();
    for (const auto& s : faces) {
        const int64_t index = static_cast<int64_t>(s.primitive_type());
        if (!m_mesh.has_child_mesh_in_dimension(index)) continue;
        global_ids_to_potential_tuples.at(index).emplace_back(
            m_mesh.id(s),
            wmtk::simplex::top_dimension_cofaces_tuples(m_mesh, s));
    }

    create_spine_simplices();
    fill_split_facet_data();
}

void TriMesh::TriMeshOperationExecutor::fill_split_facet_data()
{
    {
    const size_t facet_size = m_incident_face_datas.size();
    std::vector<int64_t> new_facet_ids = this->request_simplex_indices(PrimitiveType::Triangle, 2 * facet_size);
    assert(new_facet_ids.size() == 2 * facet_size);
    for(size_t j = 0; j < facet_size; ++j) {
        
        std::array<int64_t,2> arr;
        std::copy(new_facet_ids.begin() + 2 * j, new_facet_ids.begin() + 2*(j+1), arr.begin()
                );
        //const auto& data = 
            split_facet_data().add_facet(m_mesh, m_operating_tuple, arr);
        m_incident_face_datas[j].split_f = arr;


    }
    }
}
void TriMesh::TriMeshOperationExecutor::split_edge()
{

    split_edge_precompute();




    for (IncidentFaceData& face_data : m_incident_face_datas) {
        replace_incident_face(face_data);
    }
    assert(m_incident_face_datas.size() <= 2);
    if (m_incident_face_datas.size() > 1) {
        connect_faces_across_spine();
    }

    update_cell_hash();


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

    // return Tuple new_fid, new_vid that points
    const int64_t new_tuple_fid = m_incident_face_datas[0].split_f[1];
    if(m_mesh.is_free()) {
    m_output_tuple =
        m_mesh.tuple_from_global_ids(new_tuple_fid, split_spine_eids[1], m_free_split_v[1]);
    assert(m_mesh.id_vertex(m_output_tuple) == m_free_split_v[1]);
    } else {
    m_output_tuple =
        m_mesh.tuple_from_global_ids(new_tuple_fid, split_spine_eids[1], split_new_vid);
    assert(m_mesh.id_vertex(m_output_tuple) == split_new_vid);
    }
    assert(m_mesh.id_face(m_output_tuple) == new_tuple_fid);
    assert(m_mesh.is_valid(m_output_tuple));

    delete_simplices();
}

    void TriMesh::TriMeshOperationExecutor::create_spine_simplices() {
    if(m_mesh.is_free()) {
        // create new vertex (center)
        std::vector<int64_t> new_vids = this->request_simplex_indices(PrimitiveType::Vertex, 3);
        assert(new_vids.size() == 3);

        std::copy(new_vids.begin()+1, new_vids.end(), m_free_split_v.begin());
        split_new_vid = new_vids[0];

        // create new edges (spine)
        std::vector<int64_t> new_eids = this->request_simplex_indices(PrimitiveType::Edge, 4);
        assert(new_eids.size() == 4);

        auto midpoint = new_eids.begin() + 2;
        std::copy(new_eids.begin(), midpoint, split_spine_eids.begin());
        std::copy(midpoint, new_eids.end(), m_free_split_e.begin());

    } else {
        // create new vertex (center)
        std::vector<int64_t> new_vids = this->request_simplex_indices(PrimitiveType::Vertex, 1);
        assert(new_vids.size() == 1);
        split_new_vid = new_vids[0];
        logger().trace("new split vid {}", split_new_vid);

        // create new edges (spine)
        std::vector<int64_t> new_eids = this->request_simplex_indices(PrimitiveType::Edge, 2);
        assert(new_eids.size() == 2);

        std::copy(new_eids.begin(), new_eids.end(), split_spine_eids.begin());
        logger().trace("new split eids {}", fmt::join(split_spine_eids,","));
    }
    }

void TriMesh::TriMeshOperationExecutor::collapse_edge_precompute()
{
    set_collapse();
    is_collapse = true;

    const simplex::SimplexCollection edge_closed_star =
        simplex::closed_star(m_mesh, simplex::Simplex::edge(m_mesh, m_operating_tuple));

    // get all faces incident to the edge
    for (const simplex::Simplex& f : edge_closed_star.simplex_vector(PrimitiveType::Triangle)) {
        m_incident_face_datas.emplace_back(get_incident_face_data(f.tuple()));
    }

    assert(m_incident_face_datas.size() <= 2);
    if (m_incident_face_datas[0].fid != m_mesh.id_face(m_operating_tuple)) {
        assert(m_incident_face_datas.size() == 2);
        std::swap(m_incident_face_datas[0], m_incident_face_datas[1]);
    }

    // update hash on all faces in the two-ring neighborhood
    simplex::SimplexCollection hash_update_region(m_mesh);
    for (const simplex::Simplex& v : edge_closed_star.simplex_vector(PrimitiveType::Vertex)) {
        const simplex::SimplexCollection v_closed_star = simplex::top_dimension_cofaces(m_mesh, v);
        hash_update_region.add(v_closed_star);
    }
    hash_update_region.sort_and_clean();

    global_ids_to_potential_tuples.resize(3);
    simplex::SimplexCollection faces(m_mesh);

    for (const simplex::Simplex& f : hash_update_region.simplex_vector(PrimitiveType::Triangle)) {

        faces.add(wmtk::simplex::faces(m_mesh, f, false));
        faces.add(f);
    }

    faces.sort_and_clean();
    for (const auto& s : faces) {
        const int64_t index = static_cast<int64_t>(s.primitive_type());
        if (!m_mesh.has_child_mesh_in_dimension(index)) continue;
        global_ids_to_potential_tuples.at(index).emplace_back(
            m_mesh.id(s),
            wmtk::simplex::top_dimension_cofaces_tuples(m_mesh, s));
    }
    simplex_ids_to_delete = get_collapse_simplices_to_delete(m_operating_tuple, m_mesh);
}


void TriMesh::TriMeshOperationExecutor::collapse_edge()
{
    collapse_edge_precompute();

    if(!m_mesh.is_free()) {
    // must collect star before changing connectivity
    const simplex::SimplexCollection v0_star =
        simplex::closed_star(m_mesh, simplex::Simplex::vertex(m_mesh, m_operating_tuple));


    connect_ears();

    const int64_t& v0 = m_spine_vids[0];
    const int64_t& v1 = m_spine_vids[1];

    // replace v0 by v1 in incident faces
    for (const simplex::Simplex& f : v0_star.simplex_vector(PrimitiveType::Triangle)) {
        const int64_t fid = m_mesh.id(f);
        bool is_fid_deleted = false;
        for (int64_t i = 0; i < m_incident_face_datas.size(); ++i) {
            if (m_incident_face_datas[i].fid == fid) {
                is_fid_deleted = true;
                break;
            }
        }
        if (is_fid_deleted) continue;
        auto fv = fv_accessor.index_access().vector_attribute(fid);
        for (int64_t i = 0; i < 3; ++i) {
            if (fv[i] == v0) {
                fv[i] = v1;
                break;
            }
        }
    }
    update_cell_hash();

    const int64_t& ret_eid = m_incident_face_datas[0].ears[1].eid;
    const int64_t& ret_vid = m_spine_vids[1];
    const int64_t& ef0 = m_incident_face_datas[0].ears[0].fid;
    const int64_t& ef1 = m_incident_face_datas[0].ears[1].fid;

    const int64_t new_tuple_fid = (ef0 > -1) ? ef0 : ef1;


    m_output_tuple = m_mesh.edge_tuple_from_id(ret_eid);

    // auto faces =
    //     simplex::top_dimension_cofaces_tuples(m_mesh, simplex::Simplex::edge(m_output_tuple));

    // assert(faces.size() == m_incident_face_datas.size());
    if (m_mesh.id_vertex(m_output_tuple) != ret_vid) {
        m_output_tuple = m_mesh.switch_vertex(m_output_tuple);
    }
    assert(m_mesh.id_vertex(m_output_tuple) == ret_vid);
    if (m_mesh.id_face(m_output_tuple) != new_tuple_fid) {
        m_output_tuple = m_mesh.switch_face(m_output_tuple);
    }
    assert(m_mesh.id_face(m_output_tuple) == new_tuple_fid);
    assert(m_mesh.is_valid(m_output_tuple));
    }

    delete_simplices();


    // return a ccw tuple from left ear if it exists, otherwise return a ccw tuple from right ear
    // return m_mesh.tuple_from_id(PrimitiveType::Vertex, v1);
}

std::vector<int64_t> TriMesh::TriMeshOperationExecutor::request_simplex_indices(
    const PrimitiveType type,
    int64_t count)
{
    return EdgeOperationData::request_simplex_indices(m_mesh, type, count);
}

} // namespace wmtk
