#include <wmtk/TriMesh.h>

#include <wmtk/utils/TupleUtils.hpp>

using namespace wmtk;

TriMesh::Tuple TriMesh::Tuple::switch_vertex(const TriMesh& m) const
{
    assert(is_valid(m));

    const int v0 = m.m_tri_connectivity[m_fid][0];
    const int v1 = m.m_tri_connectivity[m_fid][1];
    const int v2 = m.m_tri_connectivity[m_fid][2];

    Tuple loc = *this;
    switch (m_eid) {
    case 0:
        assert(m_vid == v1 || m_vid == v2);
        loc.m_vid = m_vid == v1 ? v2 : v1;
        break;
    case 1:
        assert(m_vid == v0 || m_vid == v2);
        loc.m_vid = m_vid == v0 ? v2 : v0;
        break;
    case 2:
        assert(m_vid == v0 || m_vid == v1);
        loc.m_vid = m_vid == v0 ? v1 : v0;
        break;
    }
    assert(loc.is_valid(m));

    return loc;
}

TriMesh::Tuple TriMesh::Tuple::switch_edge(const TriMesh& m) const
{
    assert(is_valid(m));

    const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
    assert(lvid == 0 || lvid == 1 || lvid == 2);

    Tuple loc = *this;
    switch (lvid) {
    case 0:
        assert(m_eid == 1 || m_eid == 2);
        loc.m_eid = m_eid == 1 ? 2 : 1;
        break;
    case 1:
        assert(m_eid == 0 || m_eid == 2);
        loc.m_eid = m_eid == 0 ? 2 : 0;
        break;
    case 2:
        assert(m_eid == 0 || m_eid == 1);
        loc.m_eid = m_eid == 0 ? 1 : 0;
        break;
    }
    assert(loc.is_valid(m));
    return loc;
}

std::optional<TriMesh::Tuple> TriMesh::Tuple::switch_face(const TriMesh& m) const
{
    assert(is_valid(m));

    const size_t v0 = m_vid;
    const size_t v1 = this->switch_vertex(m).m_vid;

    // Intersect the 1-ring of the two vertices in the edge pointed by the tuple
    std::vector<size_t> v0_fids = m.m_vertex_connectivity[v0].m_conn_tris;
    std::vector<size_t> v1_fids = m.m_vertex_connectivity[v1].m_conn_tris;

    std::sort(v0_fids.begin(), v0_fids.end());
    std::sort(v1_fids.begin(), v1_fids.end());
    std::vector<int> fids;
    std::set_intersection(
        v0_fids.begin(),
        v0_fids.end(),
        v1_fids.begin(),
        v1_fids.end(),
        std::back_inserter(fids)); // make sure this is correct
    assert(fids.size() == 1 || fids.size() == 2);

    if (fids.size() == 1) return {};

    Tuple loc = *this;

    // There is a triangle on the other side
    if (fids.size() == 2) {
        // Find the fid of the triangle on the other side
        size_t fid2 = fids[0] == m_fid ? fids[1] : fids[0];
        loc.m_fid = fid2;

        // Get sorted local indices of the two vertices in the new triangle
        size_t lv0_2 = m.m_tri_connectivity[fid2].find(v0);
        assert(lv0_2 == 0 || lv0_2 == 1 || lv0_2 == 2);
        size_t lv1_2 = m.m_tri_connectivity[fid2].find(v1);
        assert(lv1_2 == 0 || lv1_2 == 1 || lv1_2 == 2);

        if (lv0_2 > lv1_2) std::swap(lv0_2, lv1_2);

        // Assign the edge id depending on the table
        if (lv0_2 == 0 && lv1_2 == 1) {
            loc.m_eid = 2;
        } else if (lv0_2 == 1 && lv1_2 == 2) {
            loc.m_eid = 0;
        } else if (lv0_2 == 0 && lv1_2 == 2) {
            loc.m_eid = 1;
        } else {
            assert(false);
        }

        loc.update_hash(m);
    }
    assert(loc.is_valid(m));
    return loc;
}

bool TriMesh::Tuple::is_valid(const TriMesh& m) const
{
    if (m.m_vertex_connectivity[m_vid].m_is_removed || m.m_tri_connectivity[m_fid].m_is_removed)
        return false;

    // Condition 3: tuple m_hash check
    if (m_hash != m.m_tri_connectivity[m_fid].hash) return false;

#ifndef NDEBUG
    //  Condition 0: Elements exist
    assert(m_vid < m.m_vertex_connectivity.size());
    assert(m_eid <= 2);
    assert(m_fid <= m.m_tri_connectivity.size());

    // Condition 1: tid and vid are consistent
    const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
    assert(lvid == 0 || lvid == 1 || lvid == 2);

    // Condition 2: eid is valid
    const int v0 = m.m_tri_connectivity[m_fid][0];
    const int v1 = m.m_tri_connectivity[m_fid][1];
    const int v2 = m.m_tri_connectivity[m_fid][2];
    switch (m_eid) {
    case 0: assert(m_vid == v1 || m_vid == v2); break;
    case 1: assert(m_vid == v0 || m_vid == v2); break;
    case 2: assert(m_vid == v0 || m_vid == v1); break;
    default: assert(false);
    }
#endif

    return true;
}

std::array<TriMesh::Tuple, 3> TriMesh::Tuple::oriented_tri_vertices(const TriMesh& m) const
{
    std::array<TriMesh::Tuple, 3> vs;
    for (int j = 0; j < 3; j++) {
        vs[j].m_vid = m.m_tri_connectivity[m_fid][j];
        vs[j].m_eid = (j + 2) % 3;
        vs[j].m_fid = m_fid;
    }
    return vs;
}

// a valid mesh can have triangles that are is_removed == true
// it can be easier for compact later on ?
bool wmtk::TriMesh::check_mesh_connectivity_validity() const
{
    std::vector<std::vector<size_t>> conn_tris(m_vertex_connectivity.size());
    for (size_t i = 0; i < m_tri_connectivity.size(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 3; j++) conn_tris[m_tri_connectivity[i][j]].push_back(i);
    }

    for (unsigned i = 0; i < m_vertex_connectivity.size(); ++i)
        std::sort(conn_tris[i].begin(), conn_tris[i].end());

    // check conn_tets duplication, order, amount ...
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        // std::cerr << "----------------" << std::endl;

        // std::vector<size_t> temp = m_vertex_connectivity[i].m_conn_tris;
        // vector_print(temp);

        // std::cerr << "---" << std::endl;
        // vector_print(conn_tris[i]);

        assert(
            m_vertex_connectivity[i].m_conn_tris == conn_tris[i] &&
            "m_vertex_connectivity[i].m_conn_tris!=conn_tris[i]");
    }
    return true;
}

// link check, prerequisite for edge collapse
bool wmtk::TriMesh::check_link_condition(const Tuple& edge) const
{
    assert(edge.is_valid(*this));
    size_t vid1 = edge.get_vid();
    size_t vid2 = switch_vertex(edge).get_vid();
    const auto v1_conn_tris = m_vertex_connectivity[vid1].m_conn_tris;
    const auto v2_conn_tris = m_vertex_connectivity[vid2].m_conn_tris;
    std::vector<size_t> v1_v2_link;
    std::vector<size_t> v1_conn_tris_verts;
    for (size_t tri : v1_conn_tris) {
        for (int j = 0; j < 3; j++) {
            v1_conn_tris_verts.push_back(m_tri_connectivity[tri][j]);
        }
    }
    vector_unique(v1_conn_tris_verts);
    vector_erase(v1_conn_tris_verts, vid1);
    std::vector<size_t> v2_conn_tris_verts;
    for (size_t tri : v2_conn_tris) {
        for (int j = 0; j < 3; j++) {
            v2_conn_tris_verts.push_back(m_tri_connectivity[tri][j]);
        }
    }
    vector_unique(v2_conn_tris_verts);
    vector_erase(v2_conn_tris_verts, vid2);
    v1_v2_link = set_intersection(v1_conn_tris_verts, v2_conn_tris_verts);

    std::vector<size_t> edge_link;
    TriMesh::Tuple tmp_tuple = switch_face(edge).value_or(edge);
    tmp_tuple = switch_edge(tmp_tuple);
    edge_link.push_back(switch_vertex(tmp_tuple).get_vid());
    tmp_tuple = switch_edge(edge);
    edge_link.push_back(switch_vertex(tmp_tuple).get_vid());
    vector_unique(edge_link);

    return (
        v1_v2_link.size() == edge_link.size() &&
        std::equal(v1_v2_link.begin(), v1_v2_link.end(), edge_link.begin()));
}

bool TriMesh::collapse_edge(const Tuple& loc0, Tuple& new_t)
{
    if (!collapse_before(loc0)) return false;

    // get the vids
    size_t vid1 = loc0.get_vid();
    size_t vid2 = switch_vertex(loc0).get_vid();

    // record the vids that will be erased for roll backs on failure
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(2);
    old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
    old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);

    // get the fids
    auto n1_fids = m_vertex_connectivity[vid1].m_conn_tris;

    auto n2_fids = m_vertex_connectivity[vid2].m_conn_tris;

    // get the fids that will be modified
    auto n12_intersect_fids = set_intersection(n1_fids, n2_fids);
    // check if the triangles intersection is the one adjcent to the edge
    size_t test_fid1 = loc0.get_fid();
    TriMesh::Tuple loc1 = switch_face(loc0).value_or(loc0);
    size_t test_fid2 = loc1.get_fid();
    //"faces at the edge is not correct"
    assert(
        vector_contains(n12_intersect_fids, test_fid1) &&
        vector_contains(n12_intersect_fids, test_fid2));
    // now mark the vertices as removed so the assertion for tuple validity in switch operations
    // won't fail
    m_vertex_connectivity[vid1].m_is_removed = true;
    m_vertex_connectivity[vid2].m_is_removed = true;
    for (size_t fid : n12_intersect_fids) {
        m_tri_connectivity[fid].m_is_removed = true;
    }

    std::vector<size_t> n12_union_fids;
    std::set_union(
        n1_fids.begin(),
        n1_fids.end(),
        n2_fids.begin(),
        n2_fids.end(),
        std::back_inserter(n12_union_fids));

    // record the fids that will be modified/erased for roll back on failure
    vector_unique(n12_union_fids);
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris(n12_union_fids.size());

    for (int i = 0; i < old_tris.size(); i++) {
        size_t fid = n12_union_fids[i];
        old_tris[i] = std::make_pair(fid, m_tri_connectivity[fid]);
        m_tri_connectivity[fid].hash++;
    }
    // modify the triangles
    // the m_conn_tris needs to be sorted
    size_t new_vid = get_next_empty_slot_v();
    for (size_t fid : n1_fids) {
        if (m_tri_connectivity[fid].m_is_removed)
            continue;
        else {
            int j = m_tri_connectivity[fid].find(vid1);
            m_tri_connectivity[fid].m_indices[j] = new_vid;
        }
    }
    for (size_t fid : n2_fids) {
        if (vector_contains(n12_intersect_fids, fid))
            continue;
        else {
            int j = m_tri_connectivity[fid].find(vid2);
            m_tri_connectivity[fid].m_indices[j] = new_vid;
        }
    }

    // now work on vids
    // add in the new vertex
    if (new_vid < m_vertex_connectivity.size() - 1) {
        assert(m_vertex_connectivity[new_vid].m_is_removed);
        m_vertex_connectivity[new_vid].m_conn_tris.clear();
    }

    for (size_t fid : n12_union_fids) {
        if (m_tri_connectivity[fid].m_is_removed)
            continue;
        else
            m_vertex_connectivity[new_vid].m_conn_tris.push_back(fid);
    }
    // This is sorting too, and it is important to sort
    vector_unique(m_vertex_connectivity[new_vid].m_conn_tris);

    // remove the erased fids from the vertices' (the one of the triangles that is not the end
    // points of the edge) connectivity list
    std::vector<std::pair<size_t, size_t>> same_edge_vid_fid;
    for (size_t fid : n12_intersect_fids) {
        auto f_vids = m_tri_connectivity[fid].m_indices;
        for (size_t f_vid : f_vids) {
            if (f_vid != vid1 && f_vid != vid2) {
                same_edge_vid_fid.emplace_back(f_vid, fid);
                assert(vector_contains(m_vertex_connectivity[f_vid].m_conn_tris, fid));
                vector_erase(m_vertex_connectivity[f_vid].m_conn_tris, fid);
            }
        }
    }


    // ? ? tuples changes. this needs to be done before post check since checked are done on tuples
    // update the old tuple version number
    // create an edge tuple for each changed edge
    // call back check will be done on this vector of tuples

    assert(m_vertex_connectivity[new_vid].m_conn_tris.size() != 0);

    const size_t gfid = m_vertex_connectivity[new_vid].m_conn_tris[0];
    int j = m_tri_connectivity[gfid].find(new_vid);
    new_t = Tuple(new_vid, (j + 2) % 3, gfid, *this);
    if (!collapse_after(new_t)) {
        // if call back check failed roll back
        // restore the changes for connected triangles and vertices
        // resotre the version-number
        // removed restore to false
        for (auto rollback : old_tris) {
            size_t fid = rollback.first;
            m_tri_connectivity[fid] = rollback.second;
        }
        for (auto rollback : old_vertices) {
            size_t vid = rollback.first;
            m_vertex_connectivity[vid] = rollback.second;
        }
        m_vertex_connectivity[new_vid].m_conn_tris.clear();
        m_vertex_connectivity[new_vid].m_is_removed = true;
        for (auto vid_fid : same_edge_vid_fid) {
            size_t vid = vid_fid.first;
            size_t fid = vid_fid.second;
            m_vertex_connectivity[vid].m_conn_tris.push_back(fid);
            std::sort(
                m_vertex_connectivity[vid].m_conn_tris.begin(),
                m_vertex_connectivity[vid].m_conn_tris.end());
        }
        for (size_t fid : n12_intersect_fids) {
            m_tri_connectivity[fid].m_is_removed = false;
        }

        // by the end the new_t and old t both exist and both valid
        return false;
    }
    return true;
}

bool TriMesh::split_edge(const Tuple& t, Tuple& new_t)
{
    throw "Not implemented";
}

void TriMesh::swap_edge(const Tuple& t, int type)
{
    throw "Not implemented";
}

void TriMesh::consolidate_mesh_connectivity()

{
    // Finds used and unused slots for vertices
    size_t f = 0;
    size_t b = m_vertex_connectivity.size() - 1;

    std::vector<size_t> v_free;
    std::vector<size_t> v_used;
    v_free.reserve(m_vertex_connectivity.size());
    v_used.reserve(m_vertex_connectivity.size());

    while (f < b) {
        // Find the first empty slot
        while ((m_vertex_connectivity[f].m_is_removed == false) && (f < b)) f++;

        // Find the last filled slot
        while ((m_vertex_connectivity[b].m_is_removed == true) && (f < b)) b--;

        // Move it
        if (f < b) {
            v_free.push_back(f++);
            v_used.push_back(b--);
        }
    }

    // Finds used and unused slots for vertices
    f = 0;
    b = m_tri_connectivity.size() - 1;

    std::vector<size_t> t_free;
    std::vector<size_t> t_used;
    v_free.reserve(m_tri_connectivity.size());
    v_used.reserve(m_tri_connectivity.size());

    while (f < b) {
        // Find the first empty slot
        while ((m_tri_connectivity[f].m_is_removed == false) && (f < b)) f++;

        // Find the last filled slot
        while ((m_tri_connectivity[b].m_is_removed == true) && (f < b)) b--;

        // Move it
        if (f < b) {
            t_free.push_back(f++);
            t_used.push_back(b--);
        }
    }

    // Map from old vertices to new ones
    std::vector<size_t> map_v(m_vertex_connectivity.size());
    for (unsigned i = 0; i < m_vertex_connectivity.size(); ++i) map_v[i] = i;

    // Map from old triangles to new ones
    std::vector<size_t> map_f(m_tri_connectivity.size());
    for (unsigned i = 0; i < m_tri_connectivity.size(); ++i) map_f[i] = i;

    // Copy the vertex attributes and update connectivity
    for (unsigned i = 0; i < v_free.size(); ++i) {
        unsigned from = v_used[i];
        unsigned to = v_free[i];

        assert(m_vertex_connectivity[from].m_is_removed == false);
        assert(m_vertex_connectivity[to].m_is_removed == true);

        m_vertex_connectivity[to] = m_vertex_connectivity[from];
        m_vertex_connectivity[from].m_is_removed = true;

        assert(m_vertex_connectivity[from].m_is_removed == true);
        assert(m_vertex_connectivity[to].m_is_removed == false);

        map_v[from] = to;
        move_vertex_attribute(from, to);
    }

    // Copy the tris attributes first
    for (unsigned i = 0; i < t_free.size(); ++i) {
        unsigned from = t_used[i];
        unsigned to = t_free[i];

        assert(m_tri_connectivity[from].m_is_removed == false);
        assert(m_tri_connectivity[to].m_is_removed == true);

        m_tri_connectivity[to] = m_tri_connectivity[from];
        m_tri_connectivity[from].m_is_removed = true;

        assert(m_tri_connectivity[from].m_is_removed == true);
        assert(m_tri_connectivity[to].m_is_removed == false);

        map_f[from] = to;
        move_face_attribute(from, to);
    }

    // vector_print(map_f);

    // Update vertex connectivity
    for (unsigned i = 0; i < m_vertex_connectivity.size(); ++i) {
        for (unsigned j = 0; j < m_vertex_connectivity[i].m_conn_tris.size(); ++j) {
            m_vertex_connectivity[i].m_conn_tris[j] =
                map_f[m_vertex_connectivity[i].m_conn_tris[j]];
        }
        std::sort(
            m_vertex_connectivity[i].m_conn_tris.begin(),
            m_vertex_connectivity[i].m_conn_tris.end());
    }

    // Update triangle connectivity
    for (unsigned i = 0; i < m_tri_connectivity.size(); ++i) {
        for (unsigned j = 0; j < m_tri_connectivity[i].m_indices.size(); ++j) {
            m_tri_connectivity[i].m_indices[j] = map_v[m_tri_connectivity[i].m_indices[j]];
        }
    }

    // DP: remember to move edge attributes too!
    f = 0;
    while (m_vertex_connectivity[f].m_is_removed == false) ++f;
    m_vertex_connectivity.resize(f);

    f = 0;
    while (m_tri_connectivity[f].m_is_removed == false) ++f;
    m_tri_connectivity.resize(f);

    // Resize user class attributes
    resize_attributes(
        m_vertex_connectivity.size(),
        m_tri_connectivity.size() * 3,
        m_tri_connectivity.size());

    // DP: remember to compact the tbb vectors!
    // m_vertex_connectivity.compact();

    assert(check_mesh_connectivity_validity());
}

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_one_ring_tris_for_vertex(
    const wmtk::TriMesh::Tuple& t) const
{
    std::vector<TriMesh::Tuple> one_ring;
    size_t vid = t.get_vid();
    auto conn_tri = m_vertex_connectivity[vid].m_conn_tris;
    for (size_t tri : conn_tri) {
        int j = m_tri_connectivity[tri].find(vid);
        one_ring.emplace_back(vid, (j + 2) % 3, tri, *this);
        assert(one_ring[one_ring.size() - 1].is_valid(*this));
    }

    return one_ring;
}

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_one_ring_edges_for_vertex(
    const wmtk::TriMesh::Tuple& t) const
{
    std::vector<Tuple> one_ring_edges;
    std::vector<size_t> one_ring_vertices;
    size_t vid = t.get_vid();
    auto one_ring_tris = get_one_ring_tris_for_vertex(t);
    for (auto tri : one_ring_tris) {
        // find the vertex
        while (tri.get_vid() != vid) {
            tri = tri.switch_vertex(*this).switch_edge(*this);
        }

        // push first edge if not there
        if (!vector_contains(one_ring_vertices, tri.switch_vertex(*this).get_vid())) {
            one_ring_vertices.push_back(tri.switch_vertex(*this).get_vid());
            one_ring_edges.push_back(tri.switch_vertex(*this));
        }

        // push second edge if not there
        tri = tri.switch_edge(*this);
        if (!vector_contains(one_ring_vertices, tri.switch_vertex(*this).get_vid())) {
            one_ring_vertices.push_back(tri.switch_vertex(*this).get_vid());
            one_ring_edges.push_back(tri.switch_vertex(*this));
        }
    }

    assert(one_ring_vertices.size() == one_ring_edges.size());

    return one_ring_edges;
}

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_oriented_vertices_for_tri(
    const wmtk::TriMesh::Tuple& t) const
{
    std::vector<TriMesh::Tuple> incident_verts;
    size_t fid = t.get_fid();
    auto indices = m_tri_connectivity[fid].m_indices;

    incident_verts.emplace_back(indices[0], 2, fid, *this);
    incident_verts.emplace_back(indices[1], 0, fid, *this);
    incident_verts.emplace_back(indices[2], 1, fid, *this);
    return incident_verts;
}

void TriMesh::create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris)
{
    m_vertex_connectivity.resize(n_vertices);
    m_tri_connectivity.resize(tris.size());
    size_t hash_cnt = 0;
    for (int i = 0; i < tris.size(); i++) {
        m_tri_connectivity[i].m_indices = tris[i];
        m_tri_connectivity[i].hash = hash_cnt;
        hash_cnt++;
        for (int j = 0; j < 3; j++) m_vertex_connectivity[tris[i][j]].m_conn_tris.push_back(i);
    }
}

std::vector<TriMesh::Tuple> TriMesh::get_vertices() const
{
    const TriMesh& m = *this;
    const size_t n_vertices = m_vertex_connectivity.size();
    std::vector<Tuple> all_vertices_tuples;
    all_vertices_tuples.resize(n_vertices);
    for (size_t i = 0; i < n_vertices; i++) {
        const std::vector<size_t>& v_conn_fids = m_vertex_connectivity[i].m_conn_tris;
        size_t fid = *min_element(v_conn_fids.begin(), v_conn_fids.end());

        // get the 3 vid
        const std::array<size_t, 3> f_conn_verts = m_tri_connectivity[fid].m_indices;
        assert(i == f_conn_verts[0] || i == f_conn_verts[1] || i == f_conn_verts[2]);
        size_t eid;
        // eid is the same as the lvid
        if (i == f_conn_verts[0]) eid = 2;
        if (i == f_conn_verts[1])
            eid = 0;
        else
            eid = 1;

        Tuple v_tuple = Tuple(i, eid, fid, m);
        assert(v_tuple.is_valid(m));
        all_vertices_tuples[i] = v_tuple;
    }
    return all_vertices_tuples;
}

std::vector<TriMesh::Tuple> TriMesh::get_faces() const
{
    const TriMesh& m = *this;
    std::vector<Tuple> all_faces_tuples;
    all_faces_tuples.resize(m.m_tri_connectivity.size());
    for (size_t i = 0; i < m.m_tri_connectivity.size(); i++) {
        // get the 3 vid
        const std::array<size_t, 3>& f_conn_verts = m.m_tri_connectivity[i].m_indices;
        size_t vid = f_conn_verts[0];
        Tuple f_tuple = Tuple(vid, 2, i, m);
        assert(f_tuple.is_valid(m));
        all_faces_tuples[i] = f_tuple;
    }
    return all_faces_tuples;
}

std::vector<TriMesh::Tuple> TriMesh::get_edges() const
{
    const TriMesh& m = *this;
    std::vector<Tuple> all_edges_tuples;
    all_edges_tuples.reserve(m.m_tri_connectivity.size() * 3 / 2);
    for (size_t i = 0; i < m.m_tri_connectivity.size(); i++) {
        for (int j = 0; j < 3; j++) {
            size_t vid = m.m_tri_connectivity[i].m_indices[j];
            size_t eid = (j + 2) % 3;
            Tuple e_tuple = Tuple(vid, eid, i, m);
            assert(e_tuple.is_valid(m));
            Tuple e_tuple2 = e_tuple.switch_face(m).value_or(e_tuple);
            assert(e_tuple2.is_valid(m));
            // return itself if it is a boundary triangle
            size_t fid2 = e_tuple2.get_fid();
            if (fid2 < i)
                continue;
            else
                all_edges_tuples.push_back(e_tuple);
        }
    }
    return all_edges_tuples;
}

size_t TriMesh::get_next_empty_slot_t()
{
    m_tri_connectivity.emplace_back();
    resize_attributes(
        m_vertex_connectivity.size(),
        m_tri_connectivity.size() * 3,
        m_tri_connectivity.size());
    return m_tri_connectivity.size() - 1;
}

size_t TriMesh::get_next_empty_slot_v()
{
    m_vertex_connectivity.emplace_back();
    resize_attributes(
        m_vertex_connectivity.size(),
        m_tri_connectivity.size() * 3,
        m_tri_connectivity.size());
    return m_vertex_connectivity.size() - 1;
}

bool TriMesh::check_manifold(const Tuple& t) const
{
    auto v1_conn_tris = m_vertex_connectivity[t.get_vid()].m_conn_tris;
    auto v2_conn_tris = m_vertex_connectivity[t.switch_vertex(*this).get_vid()].m_conn_tris;

    size_t fid1 = t.get_fid();
    size_t fid2 = switch_face(t).value_or(t).get_fid();

    vector_erase(v1_conn_tris, fid1);
    vector_erase(v1_conn_tris, fid2);
    vector_erase(v2_conn_tris, fid1);
    vector_erase(v2_conn_tris, fid2);
    return (v1_conn_tris.size() + v2_conn_tris.size() > 0);
}
