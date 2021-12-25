//
// Created by Yixin Hu on 11/3/21.
//

#include <wmtk/TetMesh.h>

#include <wmtk/utils/TupleUtils.hpp>

// DP: I do not understand the logic here
int wmtk::TetMesh::find_next_empty_slot_t() // todo: always append in the end
{
    for (int i = m_t_empty_slot; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) {
            m_t_empty_slot = i + 1;
            return i;
        }
    }
    m_tet_connectivity.emplace_back();
    m_tet_connectivity.back().timestamp = -1;
    return m_tet_connectivity.size() - 1;
}

int wmtk::TetMesh::find_next_empty_slot_v()
{
    for (int i = m_v_empty_slot; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) {
            m_v_empty_slot = i + 1;
            return i;
        }
    }
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
            edges.push_back(tuple_from_edge(i, j));
        }
    }

    unique_edge_tuples(*this, edges);

    return edges;
}

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_faces() const
{
    auto faces = std::vector<TetMesh::Tuple>();
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++) {
            auto face_t = tuple_from_face(i, j);
            auto oppo_t = switch_tetrahedron(face_t);
            if (oppo_t.has_value()) {
                // use the half-face with the smaller tid
                if (face_t.tid(*this) > oppo_t->tid(*this)) { //
                    continue;
                }
            }
            faces.emplace_back(face_t);
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


std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_vertices() const
{
    std::vector<TetMesh::Tuple> edges;
    for (auto i = 0; i < m_vertex_connectivity.size(); i++) {
        auto& vc = m_vertex_connectivity[i];
        if (vc.m_is_removed) continue;
        assert(!vc.m_conn_tets.empty());
        auto tid = vc[0];
        auto local_vid = m_tet_connectivity[tid].find(i);

        // note: the following conversion of local_vid-eid is **heavily** dependent on the specifics
        // of `m_local_edges`
        auto local_eid = local_vid;
        if (local_vid >= 2) local_eid = 5;
        edges.emplace_back(tuple_from_edge(tid, local_eid));
        if (local_vid == 3) edges.back() = switch_vertex(edges.back());

        assert(edges.back().vid(*this) == i);
        assert(edges.back().is_valid(*this));
    }
    return edges;
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

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_conn_tets(const Tuple& t) const
{
    std::vector<Tuple> tets;
    for (int t_id : m_vertex_connectivity[t.m_global_vid].m_conn_tets) {
        tets.emplace_back(tuple_from_tet(t_id));
    }
    return tets;
}

void wmtk::TetMesh::consolidate_mesh_connectivity()
{
    int v_cnt = 0;
    std::vector<size_t> map_v_ids(m_vertex_connectivity.size(), -1);
    for (int i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        map_v_ids[i] = v_cnt;
        v_cnt++;
    }
    int t_cnt = 0;
    std::vector<size_t> map_t_ids(m_tet_connectivity.size(), -1);
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        map_t_ids[i] = t_cnt;
        t_cnt++;
    }

#ifdef WILDMESHING_TOOLKIT_WITH_TBB
    tbb::concurrent_vector<VertexConnectivity> new_m_vertex_connectivity(v_cnt);
    tbb::concurrent_vector<TetrahedronConnectivity> new_m_tet_connectivity(t_cnt);
#else
    std::vector<VertexConnectivity> new_m_vertex_connectivity(v_cnt);
    std::vector<TetrahedronConnectivity> new_m_tet_connectivity(t_cnt);
#endif

    v_cnt = 0;
    for (int i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;

        new_m_vertex_connectivity[v_cnt] = m_vertex_connectivity[i];
        for (size_t& t_id : new_m_vertex_connectivity[v_cnt].m_conn_tets) t_id = map_t_ids[t_id];
        v_cnt++;
    }
    t_cnt = 0;
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;

        new_m_tet_connectivity[t_cnt] = m_tet_connectivity[i];
        for (size_t& v_id : new_m_tet_connectivity[t_cnt].m_indices) v_id = map_v_ids[v_id];
        t_cnt++;
    }

    m_vertex_connectivity = new_m_vertex_connectivity;
    m_tet_connectivity = new_m_tet_connectivity;

    check_mesh_connectivity_validity();
}