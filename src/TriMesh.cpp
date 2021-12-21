#include <wmtk/TriMesh.h>

#include <wmtk/VectorUtils.h>
#include <wmtk/TupleUtils.hpp>
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

        loc.m_version_number = m.m_tri_connectivity[loc.m_fid].version_number;
    }

    assert(loc.is_valid(m));
    return loc;
}

size_t TriMesh::find_next_empty_slot_t()
{
    // check removed of the elements on hole_list
    size_t new_fid = m_tri_connectivity.size();
    if (hole_list_t.size() != 0) {
        size_t fid_candidate = hole_list_t.back();
        hole_list_t.pop_back();
        assert(m_tri_connectivity[fid_candidate].m_is_removed);
        new_fid = fid_candidate;
    }
    return new_fid;
}

size_t TriMesh::find_next_empty_slot_v()
{
    // check removed of the elements on hole_list
    size_t new_vid = m_vertex_connectivity.size();
    if (hole_list_v.size() != 0) {
        size_t vid_candidate = hole_list_v.back();
        hole_list_v.pop_back();
        assert(m_vertex_connectivity[vid_candidate].m_is_removed);
        new_vid = vid_candidate;
    }
    return new_vid;
}

bool TriMesh::collapse_edge(const Tuple& loc0, std::vector<Tuple>& new_edges)
{
    if (!collapse_before(loc0)) return false; // what is checked at pre and post screenings?
    // get the vids
    size_t vid1 = loc0.get_vid();
    size_t vid2 = switch_vertex(loc0).get_vid();

    // record the vids that will be erased for roll backs on failure
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(2);
    old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
    old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);

    // get the fids and mark the vertices as removed
    auto n1_fids = m_vertex_connectivity[vid1].m_conn_tris;
    m_vertex_connectivity[vid1].m_is_removed = true;
    auto n2_fids = m_vertex_connectivity[vid2].m_conn_tris;
    m_vertex_connectivity[vid2].m_is_removed = true;
    // get the fids that will be modified
    auto n12_intersect_fids = set_intersection(n1_fids, n2_fids);
    // check if the triangles intersection is the one adjcent to the edge
    size_t test_fid1 = loc0.get_fid();
    Tuple loc1 = switch_face(loc0).value_or(loc0);
    size_t test_fid2 = loc1.get_fid();

    assert(
        ((std::count(n12_intersect_fids.begin(), n12_intersect_fids.end(), n1_fids.size()) &&
          std::count(n12_intersect_fids.begin(), n12_intersect_fids.end(), n2_fids.size())),
         "faces at the edge is not correct"));

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
        m_tri_connectivity[fid].version_number++;
    }
    // modify the triangles
    // the m_conn_tris needs to be sorted
    size_t new_vid = find_next_empty_slot_v();
    for (size_t fid : n1_fids) {
        if (std::count(n12_intersect_fids.begin(), n12_intersect_fids.end(), fid))
            continue;
        else {
            int j = m_tri_connectivity[fid].find(vid1);
            m_tri_connectivity[fid][j] = new_vid;
        }
    }
    for (size_t fid : n2_fids) {
        if (std::count(n12_intersect_fids.begin(), n12_intersect_fids.end(), fid))
            continue;
        else {
            int j = m_tri_connectivity[fid].find(vid2);
            m_tri_connectivity[fid][j] = new_vid;
        }
    }
    // change the triangles no longer exist to a hole
    for (size_t fid : n12_intersect_fids) {
        m_tri_connectivity[fid].m_is_removed = true;
        hole_list_t.push_back(fid);
    }

    // now work on vids
    // add in the new vertex
    if (new_vid < m_vertex_connectivity.size()) {
        assert(m_vertex_connectivity[new_vid].m_is_removed);
        m_vertex_connectivity[new_vid].m_conn_tris.clear();
    } else
        m_vertex_connectivity.resize(new_vid);

    for (size_t fid : n12_union_fids) {
        if (std::count(n12_intersect_fids.begin(), n12_intersect_fids.end(), fid))
            continue;
        else
            m_vertex_connectivity[new_vid].m_conn_tris.push_back(fid);
    }
    // remove the erased fids from the corresponding vertices' connectivity list
    std::vector<std::pair<size_t, size_t>> vid_same_edge_fid;
    for (size_t fid : n12_intersect_fids) {
        auto f_vids = m_tri_connectivity[fid].m_indices;
        for (size_t f_vid : f_vids) {
            if (f_vid != vid1 && f_vid != vid2) {
                vid_same_edge_fid.push_back(std::make_pair(f_vid, fid));
                auto conn_tris = m_vertex_connectivity[f_vid].m_conn_tris;
                assert(std::count(conn_tris.begin(), conn_tris.end(), fid));
                vector_erase(conn_tris, fid);
            }
        }
    }
    // change the vertices no longer exist to a hole
    hole_list_v.push_back(vid1);
    hole_list_v.push_back(vid2);

    // ? ? tuples changes. this needs to be done before post check since checked are done on tuples
    // update the old tuple version number
    // create an edge tuple for each changed edge
    // call back check will be done on this vector of tuples
    // #loc0.update_version_number(*this);

    for (auto vid_fid : vid_same_edge_fid) {
        size_t vid = vid_fid.first;
        size_t fid = vid_fid.second;
        int j = m_tri_connectivity[fid].find(new_vid);
        Tuple e_tuple = Tuple(new_vid, (j + 2) % 3, fid, *this);
        assert(e_tuple.is_valid(*this));
        Tuple e_tuple2 =
            switch_face(e_tuple).value_or(e_tuple); // return itself if it is a boundary triangle
        size_t fid2 = e_tuple2.get_fid();
        if (fid2 < fid)
            new_edges.push_back(e_tuple2);
        else
            new_edges.push_back(e_tuple);
    }

    if (!collapse_after(new_edges)) {
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
            hole_list_v.pop_back();
        }
        for (auto vid_fid : vid_same_edge_fid) {
            size_t vid = vid_fid.first;
            size_t fid = vid_fid.second;
            auto conn_tris = m_vertex_connectivity[vid].m_conn_tris;
            conn_tris.push_back(fid);
            std::sort(conn_tris.begin(), conn_tris.end());
        }
        for (auto fid : n12_intersect_fids) hole_list_t.pop_back();
        // loc0.update_version_number(*this);
        return false;
    }
    return true;
}