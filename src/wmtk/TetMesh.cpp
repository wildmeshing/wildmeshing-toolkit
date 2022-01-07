//
// Created by Yixin Hu on 11/3/21.
//

#include <wmtk/TetMesh.h>

#include <wmtk/utils/TupleUtils.hpp>

int wmtk::TetMesh::find_next_empty_slot_t()
{
    m_tet_connectivity.emplace_back();
    m_tet_connectivity.back().timestamp = -1;
    return m_tet_connectivity.size() - 1;
}

int wmtk::TetMesh::find_next_empty_slot_v()
{
    m_vertex_connectivity.emplace_back();
    return m_vertex_connectivity.size() - 1;
}

void wmtk::TetMesh::init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets)
{
    m_vertex_connectivity.resize(n_vertices);
    m_tet_connectivity.resize(tets.size());
    for (int i = 0; i < tets.size(); i++) {
        m_tet_connectivity[i].m_indices = tets[i];
        for (int j = 0; j < 4; j++) {
            assert(tets[i][j] < m_vertex_connectivity.size());
            m_vertex_connectivity[tets[i][j]].m_conn_tets.push_back(i);
        }
    }
}


std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_edges() const
{
    std::vector<TetMesh::Tuple> edges;
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 6; j++) {
            auto tup = tuple_from_edge(i, j);
            if (tup.eid(*this) == 6 * i + j) edges.emplace_back(tup);
        }
    }

    return edges;
}

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_faces() const
{
    auto faces = std::vector<TetMesh::Tuple>();
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++) {
            auto face_t = tuple_from_face(i, j);
            if (face_t.fid(*this) == 4 * i + j) faces.emplace_back(face_t);
        }
    }

    return faces;
}


bool wmtk::TetMesh::check_mesh_connectivity_validity() const
{
    std::vector<std::vector<size_t>> conn_tets(m_vertex_connectivity.size());
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++) conn_tets[m_tet_connectivity[i][j]].push_back(i);
    }


    for (auto& tets : conn_tets) {
        auto tmp = tets;
        vector_unique(tets);
        assert(tmp == tets);
    }

    // check conn_tets duplication, order, amount ...
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        assert(!m_vertex_connectivity[i].m_conn_tets.empty());
        assert(m_vertex_connectivity[i].m_conn_tets == conn_tets[i]);
    }

    // check is_removed
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++)
            assert(
                !m_vertex_connectivity[m_tet_connectivity[i][j]].m_is_removed &&
                "m_vertex_connectivity[m_tet_connectivity[i][j]].m_is_removed");
    }
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        for (int tid : m_vertex_connectivity[i].m_conn_tets)
            assert(!m_tet_connectivity[tid].m_is_removed && "m_tet_connectivity[tid].m_is_removed");
    }

    // check tuple
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        Tuple loc = tuple_from_vertex(i);
        check_tuple_validity(loc);
        //
        Tuple locv = switch_vertex(loc);
        Tuple loce = switch_edge(loc);
        Tuple locf = switch_face(loc);
        auto loct = switch_tetrahedron(loc);
        check_tuple_validity(locv);
        check_tuple_validity(loce);
        check_tuple_validity(locf);
        if (loct.has_value()) {
            check_tuple_validity(loct.value());
        }
    }
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        Tuple loc = tuple_from_tet(i);
        check_tuple_validity(loc);
        //
        Tuple locv = switch_vertex(loc);
        Tuple loce = switch_edge(loc);
        Tuple locf = switch_face(loc);
        auto loct = switch_tetrahedron(loc);
        check_tuple_validity(locv);
        check_tuple_validity(loce);
        check_tuple_validity(locf);
        if (loct.has_value()) {
            check_tuple_validity(loct.value());
        }
    }

    return true;
}


std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_tets() const
{
    std::vector<TetMesh::Tuple> tets;
    for (auto i = 0; i < m_tet_connectivity.size(); i++) {
        auto& t = m_tet_connectivity[i];
        if (t.m_is_removed) continue;
        tets.emplace_back(tuple_from_tet(i));

        assert(tets.back().tid(*this) == i);
        assert(tets.back().is_valid(*this));
    }
    return tets;
}


std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_vertices() const
{
    std::vector<TetMesh::Tuple> verts;
    for (auto i = 0; i < m_vertex_connectivity.size(); i++) {
        auto& vc = m_vertex_connectivity[i];
        if (vc.m_is_removed) continue;
        assert(!vc.m_conn_tets.empty());

        verts.emplace_back(tuple_from_vertex(i));

        assert(verts.back().vid(*this) == i);
        assert(verts.back().is_valid(*this));
    }
    return verts;
}

bool wmtk::TetMesh::smooth_vertex(const Tuple& loc0)
{
    if (!smooth_before(loc0)) return false;
    if (!smooth_after(loc0)) return false;

    return true;
}


wmtk::TetMesh::Tuple wmtk::TetMesh::tuple_from_edge(int tid, int local_eid) const
{
    assert(tid >= 0 && tid < m_tet_connectivity.size());
    assert(local_eid >= 0 && local_eid < m_local_edges.size());

    int vid = m_tet_connectivity[tid][m_local_edges[local_eid][0]];
    int fid = m_map_edge2face[local_eid];
    return Tuple(*this, vid, local_eid, fid, tid);
}

wmtk::TetMesh::Tuple wmtk::TetMesh::tuple_from_face(int tid, int local_fid) const
{
    assert(tid >= 0 && tid < m_tet_connectivity.size());
    assert(local_fid >= 0 && local_fid < m_local_faces.size());

    int vid = m_tet_connectivity[tid][m_local_faces[local_fid][0]];
    int lvid1 = m_local_faces[local_fid][0];
    int lvid2 = m_local_faces[local_fid][1];
    int eid = -1;
    for (int i = 0; i < 6; i++) {
        if ((m_local_edges[i][0] == lvid1 && m_local_edges[i][1] == lvid2) ||
            (m_local_edges[i][0] == lvid2 && m_local_edges[i][1] == lvid1)) {
            eid = i;
            break;
        }
    }
    int fid = m_map_edge2face[local_fid];
    return Tuple(*this, vid, eid, local_fid, tid);
}

wmtk::TetMesh::Tuple wmtk::TetMesh::tuple_from_vertex(int vid) const
{
    assert(vid >= 0 && vid < m_vertex_connectivity.size());

    int tid = m_vertex_connectivity[vid].m_conn_tets[0];
    int j = m_tet_connectivity[tid].find(vid);
    int eid = m_map_vertex2edge[j];
    int fid = m_map_edge2face[eid];
    return Tuple(*this, vid, eid, fid, tid);
}

wmtk::TetMesh::Tuple wmtk::TetMesh::tuple_from_tet(int tid) const
{
    assert(tid >= 0 && tid < m_tet_connectivity.size());

    int vid = m_tet_connectivity[tid][0];
    int eid = m_map_vertex2edge[0];
    int fid = m_map_edge2face[eid];
    return Tuple(*this, vid, eid, fid, tid);
}


std::array<wmtk::TetMesh::Tuple, 4> wmtk::TetMesh::oriented_tet_vertices(const Tuple& t) const
{
    std::array<Tuple, 4> vs;
    for (int j = 0; j < 4; j++) {
        vs[j].m_global_vid = m_tet_connectivity[t.m_global_tid][j];
        vs[j].m_local_eid = m_map_vertex2edge[j];
        vs[j].m_local_fid = m_map_edge2face[vs[j].m_local_eid];
        vs[j].m_global_tid = t.m_global_tid;
    }
    return vs;
}

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_one_ring_tets_for_vertex(const Tuple& t) const
{
    std::vector<Tuple> tets;
    for (int t_id : m_vertex_connectivity[t.m_global_vid].m_conn_tets) {
        tets.emplace_back(tuple_from_tet(t_id));
    }
    return tets;
}

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_incident_tets_for_edge(const Tuple& t) const
{
    int v1_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][0]];
    int v2_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][1]];

    auto tids = set_intersection(
        m_vertex_connectivity[v1_id].m_conn_tets,
        m_vertex_connectivity[v2_id].m_conn_tets);
    std::vector<Tuple> tets;
    for (int t_id : tids) {
        tets.push_back(tuple_from_tet(t_id));
    }
    return tets;
}

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_one_ring_tets_for_edge(const Tuple& t) const
{
    int v1_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][0]];
    int v2_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][1]];

    auto tids = m_vertex_connectivity[v1_id].m_conn_tets;
    tids.insert(
        tids.end(),
        m_vertex_connectivity[v2_id].m_conn_tets.begin(),
        m_vertex_connectivity[v1_id].m_conn_tets.end());
    vector_unique(tids);

    std::vector<Tuple> tets;
    for (int t_id : tids) {
        tets.emplace_back(tuple_from_tet(t_id));
    }
    return tets;
}


void wmtk::TetMesh::consolidate_mesh_connectivity()
{
    auto v_cnt = 0;
    std::vector<size_t> map_v_ids(m_vertex_connectivity.size(), -1);
    for (auto i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        map_v_ids[i] = v_cnt;
        v_cnt++;
    }
    auto t_cnt = 0;
    std::vector<size_t> map_t_ids(m_tet_connectivity.size(), -1);
    for (auto i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        map_t_ids[i] = t_cnt;
        t_cnt++;
    }

    v_cnt = 0;
    for (auto i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        if (v_cnt != i) {
            assert(v_cnt < i);
            m_vertex_connectivity[v_cnt] = m_vertex_connectivity[i];
            move_vertex_attribute(i, v_cnt);
        }
        for (size_t& t_id : m_vertex_connectivity[v_cnt].m_conn_tets) t_id = map_t_ids[t_id];
        v_cnt++;
    }
    t_cnt = 0;
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;

        if (t_cnt != i) {
            assert(t_cnt < i);
            m_tet_connectivity[t_cnt] = m_tet_connectivity[i];
            m_tet_connectivity[t_cnt].timestamp = 0;
            move_tet_attribute(i, t_cnt);

            for (auto j = 0; j < 4; j++) {
                move_face_attribute(i * 4 + j, t_cnt * 4 + j);
            }
            for (auto j = 0; j < 6; j++) {
                move_edge_attribute(i * 6 + j, t_cnt * 6 + j);
            }
        }
        for (size_t& v_id : m_tet_connectivity[t_cnt].m_indices) v_id = map_v_ids[v_id];
        t_cnt++;
    }

    m_vertex_connectivity.resize(v_cnt);
    m_tet_connectivity.resize(t_cnt);

    resize_attributes(v_cnt, 6 * t_cnt, 4 * t_cnt, t_cnt);

    assert(check_mesh_connectivity_validity());
}