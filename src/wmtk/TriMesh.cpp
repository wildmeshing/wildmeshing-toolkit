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

// a valid mesh can have triangles that are is_removed == true
// it can be easier for compact later on ?
bool wmtk::TriMesh::check_mesh_connectivity_validity() const
{
    std::vector<std::vector<size_t>> conn_tris(m_vertex_connectivity.size());
    for (size_t i = 0; i < m_tri_connectivity.size(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 3; j++) conn_tris[m_tri_connectivity[i][j]].push_back(i);
    }

    // check conn_tets duplication, order, amount ...
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
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
    std::vector<size_t> v2_conn_tris_verts;
    for (size_t tri : v2_conn_tris) {
        for (int j = 0; j < 3; j++) {
            v2_conn_tris_verts.push_back(m_tri_connectivity[tri][j]);
        }
    }
    vector_unique(v2_conn_tris_verts);
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
    assert(
        ("faces at the edge is not correct",
         (vector_contains(n12_intersect_fids, test_fid1) &&
          vector_contains(n12_intersect_fids, test_fid2))));
    // now mark the vertices as removed so the assertion for tuple validity in switch operations
    // won't fail
    m_vertex_connectivity[vid1].m_is_removed = true;
    m_vertex_connectivity[vid2].m_is_removed = true;

    std::vector<size_t> n12_union_fids;
    std::set_union(
        n1_fids.begin(),
        n1_fids.end(),
        n2_fids.begin(),
        n2_fids.end(),
        std::back_inserter(n12_union_fids));

    // record the fids that will be modified/erased for roll back on failure
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
        if (vector_contains(n12_intersect_fids, fid))
            continue;
        else {
            int j = m_tri_connectivity[fid].find(vid1);
            m_tri_connectivity[fid][j] = new_vid;
        }
    }
    for (size_t fid : n2_fids) {
        if (vector_contains(n12_intersect_fids, fid))
            continue;
        else {
            int j = m_tri_connectivity[fid].find(vid2);
            m_tri_connectivity[fid][j] = new_vid;
        }
    }
    for (size_t fid : n12_intersect_fids) {
        m_tri_connectivity[fid].m_is_removed = true;
    }

    // now work on vids
    // add in the new vertex
    if (new_vid < m_vertex_connectivity.size() - 1) {
        assert(m_vertex_connectivity[new_vid].m_is_removed);
        m_vertex_connectivity[new_vid].m_conn_tris.clear();
    }

    for (size_t fid : n12_union_fids) {
        if (vector_contains(n12_intersect_fids, fid))
            continue;
        else
            m_vertex_connectivity[new_vid].m_conn_tris.push_back(fid);
    }
    // remove the erased fids from the vertices' (the one of the triangles that is not the end
    // points of the edge) connectivity list
    std::vector<std::pair<size_t, size_t>> same_edge_vid_fid;
    for (size_t fid : n12_intersect_fids) {
        auto f_vids = m_tri_connectivity[fid].m_indices;
        for (size_t f_vid : f_vids) {
            if (f_vid != vid1 && f_vid != vid2) {
                same_edge_vid_fid.emplace_back(f_vid, fid);
                auto conn_tris = m_vertex_connectivity[f_vid].m_conn_tris;
                assert(vector_contains(conn_tris, fid));
                vector_erase(conn_tris, fid);
            }
        }
    }

    // ? ? tuples changes. this needs to be done before post check since checked are done on tuples
    // update the old tuple version number
    // create an edge tuple for each changed edge
    // call back check will be done on this vector of tuples

    assert(m_vertex_connectivity[new_vid].m_conn_tris.size() != 0);
    size_t fid = m_vertex_connectivity[new_vid].m_conn_tris[0];
    int j = m_tri_connectivity[fid].find(new_vid);
    new_t = Tuple(new_vid, (j + 2) % 3, fid, *this);

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
            auto conn_tris = m_vertex_connectivity[vid].m_conn_tris;
            conn_tris.push_back(fid);
            std::sort(conn_tris.begin(), conn_tris.end());
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

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_one_ring_tris_for_vertex(
    const wmtk::TriMesh::Tuple& t) const
{
    std::vector<TriMesh::Tuple> one_ring;
    size_t vid = t.get_vid();
    auto conn_tri = m_vertex_connectivity[vid].m_conn_tris;
    for (size_t tri : conn_tri) {
        int j = m_tri_connectivity[tri].find(vid);
        one_ring.emplace_back(vid, (j + 2) % 3, tri, *this);
    }
    return one_ring;
};

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_one_ring_edges_for_vertex(
    const wmtk::TriMesh::Tuple& t) const
{
    std::vector<Tuple> one_ring_edges;
    wmtk::TriMesh::Tuple current_t = t;
    int ring_size = m_vertex_connectivity[t.get_vid()].m_conn_tris.size();
    for (int i = 0; i < ring_size; i++) {
        wmtk::TriMesh::Tuple tmp_tuple = switch_face(t).value_or(current_t);
        size_t fid = current_t.get_fid();
        if (tmp_tuple.get_fid() < current_t.get_fid()) fid = tmp_tuple.get_fid();
        int j = m_tri_connectivity[fid].find(current_t.get_vid());
        one_ring_edges.emplace_back(current_t.get_vid(), (j + 2) % 3, fid, *this);
        tmp_tuple = switch_edge(current_t);
        current_t = tmp_tuple;
    }
    return one_ring_edges;
};

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
};