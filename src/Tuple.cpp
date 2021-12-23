//
// Created by Yixin Hu on 12/7/21.
//

#include "TetMesh.h"

using namespace wmtk;

void TetMesh::Tuple::init_from_edge(const TetMesh& m, int tid, int local_eid)
{
    assert(tid >= 0 && tid < m.m_tet_connectivity.size());
    assert(local_eid >= 0 && local_eid < m_local_edges.size());

    int vid = m.m_tet_connectivity[tid][m_local_edges[local_eid][0]];
    int fid = m_map_edge2face[local_eid];
    init(vid, local_eid, fid, tid, m.m_tet_connectivity[tid].timestamp);
}

void TetMesh::Tuple::init_from_face(const TetMesh& m, int tid, int local_fid)
{
    assert(tid >= 0 && tid < m.m_tet_connectivity.size());
    assert(local_fid >= 0 && local_fid < m_local_faces.size());

    int vid = m.m_tet_connectivity[tid][m_local_faces[local_fid][0]];
    int lvid1 = m_local_faces[local_fid][0];
    int lvid2 = m_local_faces[local_fid][1];
    int eid = -1;
    for (int i = 0; i < 6; i++) {
        if (m_local_edges[i][0] == lvid1 && m_local_edges[i][1] == lvid2 ||
            m_local_edges[i][0] == lvid2 && m_local_edges[i][1] == lvid1) {
            eid = i;
            break;
        }
    }
    int fid = m_map_edge2face[local_fid];
    init(vid, eid, local_fid, tid, m.m_tet_connectivity[tid].timestamp);
}

void TetMesh::Tuple::init_from_vertex(const TetMesh& m, int vid)
{
    assert(vid >= 0 && vid < m.m_vertex_connectivity.size());

    int tid = m.m_vertex_connectivity[vid].m_conn_tets[0];
    int j = m.m_tet_connectivity[tid].find(vid);
    int eid = m_map_vertex2edge[j];
    int fid = m_map_edge2face[eid];
    init(vid, eid, fid, tid, m.m_tet_connectivity[tid].timestamp);
}

void TetMesh::Tuple::init_from_tet(const TetMesh& m, int tid)
{
    assert(tid >= 0 && tid < m.m_tet_connectivity.size());

    int vid = m.m_tet_connectivity[tid][0];
    int eid = m_map_vertex2edge[0];
    int fid = m_map_edge2face[eid];
    init(vid, eid, fid, tid, m.m_tet_connectivity[tid].timestamp);
}

bool TetMesh::Tuple::is_valid(const TetMesh& m) const
{
    if (m.m_vertex_connectivity[m_vid].m_is_removed || m.m_tet_connectivity[m_tid].m_is_removed)
        return false;

    if (m_timestamp != m.m_tet_connectivity[m_tid].timestamp) return false;
    return true;
}

void TetMesh::Tuple::print_info() const
{
    logger().trace("tuple: {} {} {} {}", m_vid, m_eid, m_fid, m_tid);
}

void TetMesh::Tuple::print_info(const TetMesh& m) const
{
    //    logger().trace("tuple: {} {} {} {}", m_vid, m_eid, m_fid, m_tid);
    //    logger().trace("tet {} {} {} {}", m.m_tet_connectivity[m_tid][0],
    //    m.m_tet_connectivity[m_tid][1],
    //                   m.m_tet_connectivity[m_tid][2], m.m_tet_connectivity[m_tid][3]);
    //    logger().trace("edge {}: {} {}", m_eid, m_local_edges[m_eid][0], m_local_edges[m_eid][1]);
    //    logger().trace("face {}: {} {} {}", m_eid, m_local_faces[m_fid][0],
    //    m_local_faces[m_fid][1],
    //                   m_local_faces[m_fid][2]);

    logger().trace("tuple: {} {} {} {}", m_vid, m_eid, m_fid, m_tid);
    logger().trace(
        "tet {} {} {} {}",
        m.m_tet_connectivity[m_tid][0],
        m.m_tet_connectivity[m_tid][1],
        m.m_tet_connectivity[m_tid][2],
        m.m_tet_connectivity[m_tid][3]);
    logger().trace("edge {}: {} {}", m_eid, m_local_edges[m_eid][0], m_local_edges[m_eid][1]);
    logger().trace(
        "face {}: {} {} {}",
        m_eid,
        m_local_faces[m_fid][0],
        m_local_faces[m_fid][1],
        m_local_faces[m_fid][2]);
}


size_t TetMesh::Tuple::vid(const TetMesh&) const
{
    return m_vid;
} // update eid and fid

size_t TetMesh::Tuple::eid(const TetMesh& m) const
{
    int v1_id = m.m_tet_connectivity[m_tid][m_local_edges[m_eid][0]];
    int v2_id = m.m_tet_connectivity[m_tid][m_local_edges[m_eid][1]];
    auto n12_t_ids = set_intersection(
        m.m_vertex_connectivity[v1_id].m_conn_tets,
        m.m_vertex_connectivity[v2_id].m_conn_tets);
    assert(!n12_t_ids.empty());

    int tid = *std::min_element(n12_t_ids.begin(), n12_t_ids.end());
    for (int j = 0; j < 6; j++) {
        int tmp_v1_id = m.m_tet_connectivity[tid][m_local_edges[j][0]];
        int tmp_v2_id = m.m_tet_connectivity[tid][m_local_edges[j][1]];
        if ((tmp_v1_id == v1_id && tmp_v2_id == v2_id) ||
            (tmp_v1_id == v2_id && tmp_v2_id == v1_id))
            return tid * 6 + j;
    }
    throw std::runtime_error("Tuple::eid() error");
}

size_t TetMesh::Tuple::fid(const TetMesh& m) const
{
    std::array<size_t, 3> v_ids = {
        {m.m_tet_connectivity[m_tid][m_local_faces[m_fid][0]],
         m.m_tet_connectivity[m_tid][m_local_faces[m_fid][1]],
         m.m_tet_connectivity[m_tid][m_local_faces[m_fid][2]]}};
    auto tmp = set_intersection(
        m.m_vertex_connectivity[v_ids[0]].m_conn_tets,
        m.m_vertex_connectivity[v_ids[1]].m_conn_tets);
    auto n12_t_ids = set_intersection(tmp, m.m_vertex_connectivity[v_ids[2]].m_conn_tets);
    assert(n12_t_ids.size() == 1 || n12_t_ids.size() == 2);

    if (n12_t_ids.size() == 1) {
        return m_tid * 4 + m_fid;
    }

    std::sort(v_ids.begin(), v_ids.end());
    int tid = *std::min_element(n12_t_ids.begin(), n12_t_ids.end());
    for (int j = 0; j < 4; j++) {
        std::array<size_t, 3> tmp_v_ids = {
            {m.m_tet_connectivity[tid][m_local_faces[j][0]],
             m.m_tet_connectivity[tid][m_local_faces[j][1]],
             m.m_tet_connectivity[tid][m_local_faces[j][2]]}};
        std::sort(tmp_v_ids.begin(), tmp_v_ids.end());
        if (tmp_v_ids == v_ids) return tid * 4 + j;
    }

    throw std::runtime_error("Tuple::fid() error");
}

size_t TetMesh::Tuple::tid(const TetMesh&) const
{
    return m_tid;
}

TetMesh::Tuple TetMesh::Tuple::switch_vertex(const TetMesh& m) const
{
    Tuple loc = *this;

    int l_vid1 = m_local_edges[m_eid][0];
    int l_vid2 = m_local_edges[m_eid][1];
    loc.m_vid = m.m_tet_connectivity[m_tid][l_vid1] == m_vid ? m.m_tet_connectivity[m_tid][l_vid2]
                                                             : m.m_tet_connectivity[m_tid][l_vid1];
    assert(loc.m_vid >= 0 && loc.m_vid < m.m_vertex_connectivity.size());

    return loc;
} // along edge

TetMesh::Tuple TetMesh::Tuple::switch_edge(const TetMesh& m) const
{
    Tuple loc = *this;
    for (int leid : m_local_edges_in_a_face[m_fid]) {
        if (leid != m_eid && (m.m_tet_connectivity[m_tid][m_local_edges[leid][0]] == m_vid ||
                              m.m_tet_connectivity[m_tid][m_local_edges[leid][1]] == m_vid)) {
            loc.m_eid = leid;
            return loc;
        }
    }
    assert(false && "switch edge failed");
    return loc;
}

TetMesh::Tuple TetMesh::Tuple::switch_face(const TetMesh& m) const
{
    Tuple loc = *this;
    int l_v1_id = m_local_edges[m_eid][0];
    int l_v2_id = m_local_edges[m_eid][1];
    for (int j = 0; j < 4; j++) {
        if (j == m_fid) continue;
        int cnt = 0;
        for (int k = 0; k < 3; k++) {
            if (m_local_faces[j][k] == l_v1_id || m_local_faces[j][k] == l_v2_id) cnt++;
            if (cnt == 2) {
                loc.m_fid = j;
                return loc;
            }
        }
    }
    assert(false && "switch face failed");
    return loc;
}

std::optional<TetMesh::Tuple> TetMesh::Tuple::switch_tetrahedron(const TetMesh& m) const
{
    // eid and fid are local, so they will be changed after switch tets
    size_t v1_id = m.m_tet_connectivity[m_tid][m_local_faces[m_fid][0]];
    size_t v2_id = m.m_tet_connectivity[m_tid][m_local_faces[m_fid][1]];
    size_t v3_id = m.m_tet_connectivity[m_tid][m_local_faces[m_fid][2]];
    auto tmp = set_intersection(
        m.m_vertex_connectivity[v1_id].m_conn_tets,
        m.m_vertex_connectivity[v2_id].m_conn_tets);
    auto n123_tids = set_intersection(tmp, m.m_vertex_connectivity[v3_id].m_conn_tets);
    assert(n123_tids.size() == 1 || n123_tids.size() == 2);

    if (n123_tids.size() == 1)
        return {};
    else {
        Tuple loc = *this;
        loc.m_tid = n123_tids[0] == m_tid ? n123_tids[1] : n123_tids[0];

        loc.m_eid = m.m_tet_connectivity[loc.m_tid].find_local_edge(
            m.m_tet_connectivity[m_tid][m_local_edges[m_eid][0]],
            m.m_tet_connectivity[m_tid][m_local_edges[m_eid][1]]);
        loc.m_fid = m.m_tet_connectivity[loc.m_tid].find_local_face(v1_id, v2_id, v3_id);
        loc.m_timestamp = m.m_tet_connectivity[loc.m_tid].timestamp;

        return loc;
    }
}

std::vector<TetMesh::Tuple> TetMesh::Tuple::get_conn_tets(const TetMesh& m) const
{
    std::vector<Tuple> tets;
    for (int t_id : m.m_vertex_connectivity[m_vid].m_conn_tets) {
        tets.emplace_back();
        tets.back().init_from_tet(m, t_id);
    }
    return tets;
}

std::array<TetMesh::Tuple, 4> TetMesh::Tuple::oriented_tet_vertices(const TetMesh& m) const
{
    std::array<Tuple, 4> vs;
    for (int j = 0; j < 4; j++) {
        vs[j].m_vid = m.m_tet_connectivity[m_tid][j];
        vs[j].m_eid = m_map_vertex2edge[j];
        vs[j].m_fid = m_map_edge2face[vs[j].m_eid];
        vs[j].m_tid = m_tid;
    }
    return vs;
}

void TetMesh::Tuple::check_validity(const TetMesh& m) const
{
#ifdef NDEBUG
    return;
#endif
    // check indices
    assert(m_tid >= 0 && m_tid < m.m_tet_connectivity.size());
    assert(m_vid >= 0 && m_vid < m.m_vertex_connectivity.size());
    assert(m_eid >= 0 && m_eid < 6);
    assert(m_fid >= 0 && m_fid < 4);

    // check existence
    assert(is_valid(m));

    // check connectivity
    auto it = std::find(
        m.m_vertex_connectivity[m_vid].m_conn_tets.begin(),
        m.m_vertex_connectivity[m_vid].m_conn_tets.end(),
        m_tid);
    assert(it != m.m_vertex_connectivity[m_vid].m_conn_tets.end());
    //
    std::array<size_t, 3> f_vids = {
        {m.m_tet_connectivity[m_tid][m_local_faces[m_fid][0]],
         m.m_tet_connectivity[m_tid][m_local_faces[m_fid][1]],
         m.m_tet_connectivity[m_tid][m_local_faces[m_fid][2]]}};
    std::array<size_t, 2> e_vids = {
        {m.m_tet_connectivity[m_tid][m_local_edges[m_eid][0]],
         m.m_tet_connectivity[m_tid][m_local_edges[m_eid][1]]}};
    assert(std::find(e_vids.begin(), e_vids.end(), m_vid) != e_vids.end());
    for (int e_vid : e_vids) assert(std::find(f_vids.begin(), f_vids.end(), e_vid) != f_vids.end());
    for (int f_vid : f_vids) assert(m.m_tet_connectivity[m_tid].find(f_vid) >= 0);
}