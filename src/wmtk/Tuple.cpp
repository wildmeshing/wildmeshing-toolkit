#include "TetMesh.h"

#include <Tracy.hpp>

using namespace wmtk;


TetMesh::Tuple::Tuple(const TetMesh& m, size_t vid, size_t local_eid, size_t local_fid, size_t tid)
    : m_global_vid(vid)
    , m_local_eid(local_eid)
    , m_local_fid(local_fid)
    , m_global_tid(tid)
    , m_hash(m.m_tet_connectivity[tid].hash)
{
    check_validity(m);
}


bool TetMesh::Tuple::is_boundary_face(const TetMesh& m) const
{
    
    auto v0 = this->vid(m);
    auto oppo = this->switch_vertex(m);
    auto v1 = oppo.vid(m);
    auto v2 = oppo.switch_edge(m).switch_vertex(m).vid(m);
    assert(v0 != v1 && v1 != v2 && v2 != v0);

    auto inter01 = set_intersection(
        m.m_vertex_connectivity[v0].m_conn_tets,
        m.m_vertex_connectivity[v1].m_conn_tets);
    auto inter012 = set_intersection(inter01, m.m_vertex_connectivity[v2].m_conn_tets);

    assert(inter012.size() == 1 || inter012.size() == 2);
    return inter012.size() == 1;
}

bool TetMesh::Tuple::is_boundary_edge(const TetMesh& m) const
{
    
    auto tet_id = this->tid(m);

    Tuple e = *this;
    auto cnt = 0;
    do {
        e = e.switch_face(m);
        logger().trace("edge {} ({}) {} {}", e.tid(m), e.fid(m), e.eid(m), e.vid(m));
        auto e_opt = e.switch_tetrahedron(m);
        if (!e_opt) {
            logger().trace(">> No adjacent tetra");
            return true;
        }
        e = e_opt.value();
        cnt++;
        logger().trace("edge ({}) {} {} {}", e.tid(m), e.fid(m), e.eid(m), e.vid(m));
        assert(cnt < m.tet_capacity() + 1 && "Debug: Avoid infinite loop.");
    } while (e.tid(m) != tet_id);
    logger().trace(">> Internal");
    return false;
}

bool TetMesh::Tuple::is_valid(const TetMesh& m) const
{
    if (m_global_tid + 1 == 0) return false;
    if (m.m_vertex_connectivity[m_global_vid].m_is_removed ||
        m.m_tet_connectivity[m_global_tid].m_is_removed)
        return false;

    if (m_hash != m.m_tet_connectivity[m_global_tid].hash) return false;
    return true;
}

void TetMesh::Tuple::print_info() const
{
    logger().trace("tuple: {} {} {} {}", m_global_vid, m_local_eid, m_local_fid, m_global_tid);
}

void TetMesh::Tuple::print_info(const TetMesh& m) const
{
    //    logger().trace("tuple: {} {} {} {}", m_global_vid, m_local_eid, m_local_fid,
    //    m_global_tid); logger().trace("tet {} {} {} {}", m.m_tet_connectivity[m_global_tid][0],
    //    m.m_tet_connectivity[m_global_tid][1],
    //                   m.m_tet_connectivity[m_global_tid][2],
    //                   m.m_tet_connectivity[m_global_tid][3]);
    //    logger().trace("edge {}: {} {}", m_local_eid, m_local_edges[m_local_eid][0],
    //    m_local_edges[m_local_eid][1]); logger().trace("face {}: {} {} {}", m_local_eid,
    //    m_local_faces[m_local_fid][0], m_local_faces[m_local_fid][1],
    //                   m_local_faces[m_local_fid][2]);

    logger().trace("tuple: {} {} {} {}", m_global_vid, m_local_eid, m_local_fid, m_global_tid);
    logger().trace(
        "tet {} {} {} {}",
        m.m_tet_connectivity[m_global_tid][0],
        m.m_tet_connectivity[m_global_tid][1],
        m.m_tet_connectivity[m_global_tid][2],
        m.m_tet_connectivity[m_global_tid][3]);
    logger().trace(
        "edge {}: {} {}",
        m_local_eid,
        m_local_edges[m_local_eid][0],
        m_local_edges[m_local_eid][1]);
    logger().trace(
        "face {}: {} {} {}",
        m_local_eid,
        m_local_faces[m_local_fid][0],
        m_local_faces[m_local_fid][1],
        m_local_faces[m_local_fid][2]);
}


size_t TetMesh::Tuple::vid(const TetMesh&) const
{
    return m_global_vid;
} // update eid and fid

size_t TetMesh::Tuple::eid(const TetMesh& m) const
{
    ZoneScoped;
    auto v1_id = m.m_tet_connectivity[m_global_tid][m_local_edges[m_local_eid][0]];
    auto v2_id = m.m_tet_connectivity[m_global_tid][m_local_edges[m_local_eid][1]];
    if (v1_id > v2_id) std::swap(v1_id, v2_id);
    auto n12_t_ids = set_intersection(
        m.m_vertex_connectivity[v1_id].m_conn_tets,
        m.m_vertex_connectivity[v2_id].m_conn_tets);
    assert(!n12_t_ids.empty());

    auto tid = *std::min_element(n12_t_ids.begin(), n12_t_ids.end());
    for (int j = 0; j < 6; j++) {
        auto tmp_v1_id = m.m_tet_connectivity[tid][m_local_edges[j][0]];
        auto tmp_v2_id = m.m_tet_connectivity[tid][m_local_edges[j][1]];
        if (tmp_v1_id > tmp_v2_id) std::swap(tmp_v1_id, tmp_v2_id);
        if (tmp_v1_id == v1_id && tmp_v2_id == v2_id)
            return tid * 6 + j;
    }
    return std::numeric_limits<size_t>::max();
}

size_t TetMesh::Tuple::fid(const TetMesh& m) const
{
    
    std::array<size_t, 3> v_ids = {
        {m.m_tet_connectivity[m_global_tid][m_local_faces[m_local_fid][0]],
         m.m_tet_connectivity[m_global_tid][m_local_faces[m_local_fid][1]],
         m.m_tet_connectivity[m_global_tid][m_local_faces[m_local_fid][2]]}};
    auto tmp = set_intersection(
        m.m_vertex_connectivity[v_ids[0]].m_conn_tets,
        m.m_vertex_connectivity[v_ids[1]].m_conn_tets);
    auto n12_t_ids = set_intersection(tmp, m.m_vertex_connectivity[v_ids[2]].m_conn_tets);
    assert(n12_t_ids.size() == 1 || n12_t_ids.size() == 2);

    if (n12_t_ids.size() == 1) {
        return m_global_tid * 4 + m_local_fid;
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

    throw std::runtime_error("Tuple::fid(*this) error");
}

size_t TetMesh::Tuple::tid(const TetMesh&) const
{
    return m_global_tid;
}

TetMesh::Tuple TetMesh::Tuple::switch_vertex(const TetMesh& m) const
{
    
    Tuple loc = *this;

    int l_vid1 = m_local_edges[m_local_eid][0];
    int l_vid2 = m_local_edges[m_local_eid][1];
    loc.m_global_vid = m.m_tet_connectivity[m_global_tid][l_vid1] == m_global_vid
                           ? m.m_tet_connectivity[m_global_tid][l_vid2]
                           : m.m_tet_connectivity[m_global_tid][l_vid1];
    assert(loc.m_global_vid >= 0 && loc.m_global_vid < m.vert_capacity());

    return loc;
} // along edge

TetMesh::Tuple TetMesh::Tuple::switch_edge(const TetMesh& m) const
{
    
    Tuple loc = *this;
    for (int leid : m_local_edges_in_a_face[m_local_fid]) {
        if (leid != m_local_eid &&
            (m.m_tet_connectivity[m_global_tid][m_local_edges[leid][0]] == m_global_vid ||
             m.m_tet_connectivity[m_global_tid][m_local_edges[leid][1]] == m_global_vid)) {
            loc.m_local_eid = leid;
            return loc;
        }
    }
    assert(false && "switch edge failed");
    return loc;
}

TetMesh::Tuple TetMesh::Tuple::switch_face(const TetMesh& m) const
{
    
    Tuple loc = *this;
    int l_v1_id = m_local_edges[m_local_eid][0];
    int l_v2_id = m_local_edges[m_local_eid][1];
    for (int j = 0; j < 4; j++) {
        if (j == m_local_fid) continue;
        int cnt = 0;
        for (int k = 0; k < 3; k++) {
            if (m_local_faces[j][k] == l_v1_id || m_local_faces[j][k] == l_v2_id) cnt++;
            if (cnt == 2) {
                loc.m_local_fid = j;
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
    size_t v1_id = m.m_tet_connectivity[m_global_tid][m_local_faces[m_local_fid][0]];
    size_t v2_id = m.m_tet_connectivity[m_global_tid][m_local_faces[m_local_fid][1]];
    size_t v3_id = m.m_tet_connectivity[m_global_tid][m_local_faces[m_local_fid][2]];
    auto tmp = set_intersection(
        m.m_vertex_connectivity[v1_id].m_conn_tets,
        m.m_vertex_connectivity[v2_id].m_conn_tets);
    auto n123_tids = set_intersection(tmp, m.m_vertex_connectivity[v3_id].m_conn_tets);
    assert(n123_tids.size() == 1 || n123_tids.size() == 2);

    if (n123_tids.size() == 1)
        return {};
    else {
        Tuple loc = *this;
        loc.m_global_tid = n123_tids[0] == m_global_tid ? n123_tids[1] : n123_tids[0];

        loc.m_local_eid = m.m_tet_connectivity[loc.m_global_tid].find_local_edge(
            m.m_tet_connectivity[m_global_tid][m_local_edges[m_local_eid][0]],
            m.m_tet_connectivity[m_global_tid][m_local_edges[m_local_eid][1]]);
        loc.m_local_fid =
            m.m_tet_connectivity[loc.m_global_tid].find_local_face(v1_id, v2_id, v3_id);
        loc.m_hash = m.m_tet_connectivity[loc.m_global_tid].hash;

        return loc;
    }
}

void TetMesh::Tuple::check_validity(const TetMesh& m) const
{
    
#ifdef NDEBUG
    return;
#endif
    // check indices
    assert(m_global_tid < m.tet_capacity());
    assert(m_global_vid < m.vert_capacity());
    assert(m_local_eid < 6);
    assert(m_local_fid < 4);

    // check existence
    assert(is_valid(m));

    //
    std::array<size_t, 3> f_vids = {
        {m.m_tet_connectivity[m_global_tid][m_local_faces[m_local_fid][0]],
         m.m_tet_connectivity[m_global_tid][m_local_faces[m_local_fid][1]],
         m.m_tet_connectivity[m_global_tid][m_local_faces[m_local_fid][2]]}};
    std::array<size_t, 2> e_vids = {
        {m.m_tet_connectivity[m_global_tid][m_local_edges[m_local_eid][0]],
         m.m_tet_connectivity[m_global_tid][m_local_edges[m_local_eid][1]]}};
    assert(std::find(e_vids.begin(), e_vids.end(), m_global_vid) != e_vids.end());
    for (int e_vid : e_vids) assert(std::find(f_vids.begin(), f_vids.end(), e_vid) != f_vids.end());
    for (int f_vid : f_vids) assert(m.m_tet_connectivity[m_global_tid].find(f_vid) >= 0);
}
