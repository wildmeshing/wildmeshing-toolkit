#include <set>
#include "ManExtractMesh.h"


namespace wmtk::components::manifold_extraction {


bool ManExtractMesh::split_edge_before(const Tuple& t)
{
    // load and reset cache
    auto& cache = edge_split_cache.local();
    cache.internal_e.clear();
    cache.external_e.clear();
    cache.link_e.clear();
    cache.split_f.clear();
    cache.internal_f.clear();
    cache.external_f.clear();
    cache.tets.clear();

    // vertices
    cache.v1_id = t.vid(*this);
    cache.v2_id = switch_vertex(t).vid(*this);
    Vector3d p1 = m_vertex_attribute[cache.v1_id].m_posf;
    Vector3d p2 = m_vertex_attribute[cache.v2_id].m_posf;
    cache.new_v = VertexAttributes((p1 + p2) / 2);
    cache.new_v.label = m_edge_attribute[t.eid(*this)].label;

    // split edge
    cache.split_e = m_edge_attribute[t.eid(*this)];

    // link edge maps (and collect opp vert ids)
    std::set<size_t> opp_verts;
    auto tets = get_incident_tets_for_edge(t); // Tuples
    const simplex::Edge edge(cache.v1_id, cache.v2_id);
    for (const Tuple& t_inc : tets) {
        const simplex::Tet tet = simplex_from_tet(t_inc);
        const simplex::Edge opp = tet.opposite_edge(edge);
        opp_verts.insert(opp.vertices()[0]);
        opp_verts.insert(opp.vertices()[1]);

        // link edge attribute
        Tuple e1 = tuple_from_edge({opp.vertices()[0], opp.vertices()[1]});
        cache.link_e[opp] = m_edge_attribute[e1.eid(*this)];

        // face attributes
        FaceAttributes new_fattr;
        new_fattr.label = m_tet_attribute[t_inc.tid(*this)].label;
        cache.internal_f[opp] = new_fattr;

        auto [_1, global_fid1] =
            tuple_from_face({opp.vertices()[0], opp.vertices()[1], cache.v1_id});
        auto [_2, global_fid2] =
            tuple_from_face({opp.vertices()[0], opp.vertices()[1], cache.v2_id});
        cache.external_f[std::make_pair(opp, cache.v1_id)] = m_face_attribute[global_fid1];
        cache.external_f[std::make_pair(opp, cache.v2_id)] = m_face_attribute[global_fid2];

        // tet attribute
        cache.tets[opp] = m_tet_attribute[t_inc.tid(*this)];
    }

    // opp vertex maps
    for (const size_t opp_vid : opp_verts) {
        // edge maps
        auto [_1, global_fid1] = tuple_from_face({opp_vid, cache.v1_id, cache.v2_id});
        EdgeAttributes new_eattr;
        new_eattr.label = m_face_attribute[global_fid1].label;
        cache.internal_e[opp_vid] = new_eattr;

        size_t glob_eid1 = tuple_from_edge({cache.v1_id, opp_vid}).eid(*this);
        size_t glob_eid2 = tuple_from_edge({cache.v2_id, opp_vid}).eid(*this);
        simplex::Edge e1(cache.v1_id, opp_vid);
        simplex::Edge e2(cache.v2_id, opp_vid);
        cache.external_e[e1] = m_edge_attribute[glob_eid1];
        cache.external_e[e2] = m_edge_attribute[glob_eid2];

        // face maps
        cache.split_f[opp_vid] = m_face_attribute[global_fid1];
    }

    return true;
}


bool ManExtractMesh::split_edge_after(const Tuple& t)
{
    if (!TetMesh::split_edge_after(t)) {
        return false;
    } // why tf do we need this?

    auto& cache = edge_split_cache.local();
    const size_t v_id = t.vid(*this); // new vertex
    const size_t v1_id = cache.v1_id;
    const size_t v2_id = cache.v2_id;

    // vertex attribute
    m_vertex_attribute[v_id] = cache.new_v;

    // split edges attribute
    size_t split_e1_id = tuple_from_edge({v1_id, v_id}).eid(*this);
    size_t split_e2_id = tuple_from_edge({v2_id, v_id}).eid(*this);
    m_edge_attribute[split_e1_id] = cache.split_e;
    m_edge_attribute[split_e2_id] = cache.split_e;

    // link edge maps
    for (const auto& pair : cache.link_e) { // for every link edge
        auto link_edge = pair.first;

        // tet attributes
        Tuple t1 = tuple_from_vids(link_edge.vertices()[0], link_edge.vertices()[1], v1_id, v_id);
        Tuple t2 = tuple_from_vids(link_edge.vertices()[0], link_edge.vertices()[1], v2_id, v_id);
        m_tet_attribute[t1.tid(*this)] = cache.tets[link_edge];
        m_tet_attribute[t2.tid(*this)] = cache.tets[link_edge];

        // face attributes
        auto [_1, glob_fid1] =
            tuple_from_face({link_edge.vertices()[0], link_edge.vertices()[1], v_id});
        m_face_attribute[glob_fid1] = cache.internal_f[link_edge];

        auto [_2, glob_fid2] =
            tuple_from_face({link_edge.vertices()[0], link_edge.vertices()[1], v1_id});
        auto [_3, glob_fid3] =
            tuple_from_face({link_edge.vertices()[0], link_edge.vertices()[1], v2_id});
        m_face_attribute[glob_fid2] = cache.external_f[std::make_pair(link_edge, v1_id)];
        m_face_attribute[glob_fid3] = cache.external_f[std::make_pair(link_edge, v2_id)];

        // edge attributes
        size_t link_e_glob_id =
            tuple_from_edge({link_edge.vertices()[0], link_edge.vertices()[1]}).eid(*this);
        m_edge_attribute[link_e_glob_id] = pair.second;
    }

    // oppo vertex maps
    for (const auto& pair : cache.internal_e) { // for every oppo vertex
        size_t opp_vid = pair.first;

        // face attributes
        auto [_1, glob_fid1] = tuple_from_face({opp_vid, v1_id, v_id});
        auto [_2, glob_fid2] = tuple_from_face({opp_vid, v2_id, v_id});
        m_face_attribute[glob_fid1] = cache.split_f[opp_vid];
        m_face_attribute[glob_fid2] = cache.split_f[opp_vid];

        // edge attributes
        size_t glob_eid = tuple_from_edge({v_id, opp_vid}).eid(*this);
        size_t glob_eid1 = tuple_from_edge({v1_id, opp_vid}).eid(*this);
        size_t glob_eid2 = tuple_from_edge({v2_id, opp_vid}).eid(*this);
        m_edge_attribute[glob_eid] = cache.internal_e[opp_vid];
        m_edge_attribute[glob_eid1] = cache.external_e[simplex::Edge(v1_id, opp_vid)];
        m_edge_attribute[glob_eid2] = cache.external_e[simplex::Edge(v2_id, opp_vid)];
    }

    return true;
}


bool ManExtractMesh::split_face_before(const Tuple& t)
{
    // load and reset cache
    auto& cache = face_split_cache.local();
    cache.existing_e.clear();
    cache.existing_f.clear();
    cache.tets.clear();

    // get split face tags (used a bunch later)
    size_t split_f_id = t.fid(*this);
    cache.splitf_label = m_face_attribute[split_f_id].label;

    // new vertex
    cache.v1_id = t.vid(*this);
    cache.v2_id = switch_vertex(t).vid(*this);
    cache.v3_id = switch_vertex(switch_edge(t)).vid(*this);

    // get 1 or 2 vertex(es) opposite to face
    std::vector<size_t> tet_ids;
    tet_ids.push_back(t.tid(*this));
    auto other_tet = switch_tetrahedron(t); // is std::optional<Tuple> object
    if (other_tet) {
        tet_ids.push_back(other_tet.value().tid(*this));
    }
    std::vector<size_t> oppo_vids;
    for (const size_t tet_id : tet_ids) {
        auto tet_vids = oriented_tet_vids(tet_id);
        for (const size_t tet_vid : tet_vids) {
            if ((tet_vid != cache.v1_id) && (tet_vid != cache.v2_id) && (tet_vid != cache.v3_id)) {
                oppo_vids.push_back(tet_vid);
            }
        }
    }
    assert(oppo_vids.size() == tet_ids.size());

    // cache tet attributes
    for (int i = 0; i < tet_ids.size(); i++) {
        cache.tets[oppo_vids[i]] = m_tet_attribute[tet_ids[i]];
    }

    // existing edge maps on split face
    size_t glob_eid1 = tuple_from_edge({cache.v1_id, cache.v2_id}).eid(*this);
    cache.existing_e[simplex::Edge(cache.v1_id, cache.v2_id)] = m_edge_attribute[glob_eid1];
    size_t glob_eid2 = tuple_from_edge({cache.v2_id, cache.v3_id}).eid(*this);
    cache.existing_e[simplex::Edge(cache.v2_id, cache.v3_id)] = m_edge_attribute[glob_eid2];
    size_t glob_eid3 = tuple_from_edge({cache.v1_id, cache.v3_id}).eid(*this);
    cache.existing_e[simplex::Edge(cache.v1_id, cache.v3_id)] = m_edge_attribute[glob_eid3];

    // existing (unmodified) edge and face maps per-tet
    for (const size_t oppo_vid : oppo_vids) {
        // edges
        size_t glob_opp_eid_1 = tuple_from_edge({cache.v1_id, oppo_vid}).eid(*this);
        cache.existing_e[simplex::Edge(cache.v1_id, oppo_vid)] = m_edge_attribute[glob_opp_eid_1];
        size_t glob_opp_eid_2 = tuple_from_edge({cache.v2_id, oppo_vid}).eid(*this);
        cache.existing_e[simplex::Edge(cache.v2_id, oppo_vid)] = m_edge_attribute[glob_opp_eid_2];
        size_t glob_opp_eid_3 = tuple_from_edge({cache.v3_id, oppo_vid}).eid(*this);
        cache.existing_e[simplex::Edge(cache.v3_id, oppo_vid)] = m_edge_attribute[glob_opp_eid_3];

        // faces
        auto [_1, glob_fid1] = tuple_from_face({cache.v1_id, cache.v2_id, oppo_vid});
        cache.existing_f[simplex::Face(cache.v1_id, cache.v2_id, oppo_vid)] =
            m_face_attribute[glob_fid1];
        auto [_2, glob_fid2] = tuple_from_face({cache.v2_id, cache.v3_id, oppo_vid});
        cache.existing_f[simplex::Face(cache.v2_id, cache.v3_id, oppo_vid)] =
            m_face_attribute[glob_fid2];
        auto [_3, glob_fid3] = tuple_from_face({cache.v1_id, cache.v3_id, oppo_vid});
        cache.existing_f[simplex::Face(cache.v1_id, cache.v3_id, oppo_vid)] =
            m_face_attribute[glob_fid3];
    }

    return true;
}


bool ManExtractMesh::split_face_after(const Tuple& t)
{
    if (!TetMesh::split_face_after(t)) {
        return false;
    } // why tf do we need this?

    auto& cache = face_split_cache.local();
    size_t v_id = vertex_size() - 1;
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;
    size_t v3_id = cache.v3_id;
    std::array<size_t, 3> splitf_vids = {v1_id, v2_id, v3_id};

    // new_vertex
    m_vertex_attribute[v_id].m_posf =
        (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf +
         m_vertex_attribute[v3_id].m_posf) /
        3;
    m_vertex_attribute[v_id].label = cache.splitf_label;

    // new edges/faces on split face
    EdgeAttributes splitf_eattr;
    splitf_eattr.label = cache.splitf_label;
    FaceAttributes splitf_fattr;
    splitf_fattr.label = cache.splitf_label;
    for (int i = 0; i < 3; i++) {
        size_t curr_v1_id = splitf_vids[i];
        size_t curr_v2_id = splitf_vids[(i + 1) % 3];

        size_t glob_eid = tuple_from_edge({curr_v1_id, v_id}).eid(*this); // new edge
        m_edge_attribute[glob_eid] = splitf_eattr;

        auto [_, glob_fid] = tuple_from_face({curr_v1_id, curr_v2_id, v_id}); // new face
        m_face_attribute[glob_fid] = splitf_fattr;
    }

    // existing edges on split face
    size_t glob_eid1 = tuple_from_edge({v1_id, v2_id}).eid(*this);
    size_t glob_eid2 = tuple_from_edge({v2_id, v3_id}).eid(*this);
    size_t glob_eid3 = tuple_from_edge({v1_id, v3_id}).eid(*this);
    m_edge_attribute[glob_eid1] = cache.existing_e[simplex::Edge(v1_id, v2_id)];
    m_edge_attribute[glob_eid2] = cache.existing_e[simplex::Edge(v2_id, v3_id)];
    m_edge_attribute[glob_eid3] = cache.existing_e[simplex::Edge(v1_id, v3_id)];

    // per oppo-vert
    for (const auto& pair : cache.tets) {
        size_t opp_vid = pair.first;

        // new edge
        size_t glob_newe_id = tuple_from_edge({v_id, opp_vid}).eid(*this);
        m_edge_attribute[glob_newe_id].label = pair.second.label;

        // every pair of existing split face verts (every split face edge)
        for (int i = 0; i < 3; i++) {
            size_t curr_v1_id = splitf_vids[i];
            size_t curr_v2_id = splitf_vids[(i + 1) % 3];

            // new tet
            Tuple tet = tuple_from_vids(v_id, opp_vid, curr_v1_id, curr_v2_id);
            m_tet_attribute[tet.tid(*this)] = pair.second;

            // new face
            auto [_, glob_fid] = tuple_from_face({v_id, curr_v1_id, opp_vid});
            m_face_attribute[glob_fid].label = pair.second.label;

            // existing face
            auto [_2, glob_fid2] = tuple_from_face({opp_vid, curr_v1_id, curr_v2_id});
            m_face_attribute[glob_fid2] =
                cache.existing_f[simplex::Face(opp_vid, curr_v1_id, curr_v2_id)];

            // existing edge
            size_t glob_eid = tuple_from_edge({curr_v1_id, opp_vid}).eid(*this);
            m_edge_attribute[glob_eid] = cache.existing_e[simplex::Edge(curr_v1_id, opp_vid)];
        }
    }

    return true;
}


bool ManExtractMesh::split_tet_before(const Tuple& t)
{
    auto& cache = tet_split_cache.local();
    cache.existing_e.clear();
    cache.existing_f.clear();

    // vertices
    auto vs = oriented_tet_vids(t);
    for (int i = 0; i < 4; i++) { // deep copy just to be safe
        cache.v_ids[i] = vs[i];
    }

    // cache retained edge attributes
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            size_t glob_eid = tuple_from_edge({cache.v_ids[i], cache.v_ids[j]}).eid(*this);
            cache.existing_e[simplex::Edge(cache.v_ids[i], cache.v_ids[j])] =
                m_edge_attribute[glob_eid];
        }
    }

    // cache retained face attributes
    for (int i = 0; i < 4; i++) {
        size_t v1 = cache.v_ids[i];
        size_t v2 = cache.v_ids[(i + 1) % 4];
        size_t v3 = cache.v_ids[(i + 2) % 4];
        auto [_, glob_fid] = tuple_from_face({v1, v2, v3});
        cache.existing_f[simplex::Face(v1, v2, v3)] = m_face_attribute[glob_fid];
    }

    // tet attribute
    cache.tet = m_tet_attribute[t.tid(*this)];

    return true;
}

bool ManExtractMesh::split_tet_after(const Tuple& t)
{
    if (!TetMesh::split_tet_after(t)) {
        return false;
    } // why tf do we need this?

    auto& cache = tet_split_cache.local();
    int tet_label = cache.tet.label;
    size_t v_id = vertex_size() - 1;

    // new vertex
    m_vertex_attribute[v_id].m_posf =
        (m_vertex_attribute[cache.v_ids[0]].m_posf + m_vertex_attribute[cache.v_ids[1]].m_posf +
         m_vertex_attribute[cache.v_ids[2]].m_posf + m_vertex_attribute[cache.v_ids[3]].m_posf) /
        4;
    m_vertex_attribute[v_id].label = tet_label;

    // iterate over new tets (retained faces, new tets, new edge (opposite tet) )
    for (int i = 0; i < 4; i++) {
        size_t v1 = cache.v_ids[i];
        size_t v2 = cache.v_ids[(i + 1) % 4];
        size_t v3 = cache.v_ids[(i + 2) % 4];

        // new edge (doesn't matter which vertex, will iterate through all 4)
        size_t glob_new_eid = tuple_from_edge({v_id, v1}).eid(*this);
        m_edge_attribute[glob_new_eid].label = tet_label;

        // retained face
        auto [_, glob_fid] = tuple_from_face({v1, v2, v3});
        m_face_attribute[glob_fid] = cache.existing_f[simplex::Face(v1, v2, v3)];

        // new tet
        size_t t_id = tuple_from_vids(v1, v2, v3, v_id).tid(*this);
        m_tet_attribute[t_id] = cache.tet; // TO VERIFY: this creates deep copy?
    }

    // existing edges and new faces
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            size_t v1 = cache.v_ids[i];
            size_t v2 = cache.v_ids[j];

            // existing edge
            size_t glob_eid = tuple_from_edge({v1, v2}).eid(*this);
            m_edge_attribute[glob_eid] = cache.existing_e[simplex::Edge(v1, v2)];

            // new face
            auto [_, glob_fid] = tuple_from_face({v1, v2, v_id});
            m_face_attribute[glob_fid].label = tet_label;
        }
    }

    return true;
}

} // namespace wmtk::components::manifold_extraction