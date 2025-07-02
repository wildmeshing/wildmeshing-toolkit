#include <wmtk/TetMesh.h>


namespace wmtk {

bool TetMesh::split_tet(const Tuple& t, std::vector<Tuple>& new_tets)
{
    if (!split_tet_before(t)) {
        return false;
    }
    const SmartTuple tt(*this, t);
    if (!tt.is_valid()) {
        return false;
    }

    /**
     *
     *
     *         v2
     *         /|\
     *        / | \
     *       /  |  \
     *      /t1 X t0\
     *     / /(t3)\  \
     *    //   t2    \\
     *  v0 ----------- v1
     *
     * - X is the new vertex
     * - v3 is above X (not displayed)
     * - t3 is below the new vertex
     *
     * New tet vertex connectivity:
     * t0: (X,1,2,3) <- modified old tet
     * t1: (0,X,2,3)
     * t2: (0,1,X,3)
     * t3: (0,1,2,X)
     */
    const size_t tid = tt.tid();

    std::array<size_t, 4> vid;
    vid[0] = tt.vid();
    vid[1] = tt.switch_vertex().vid();
    vid[2] = tt.switch_edge().switch_vertex().vid();
    vid[3] = tt.switch_face().switch_edge().switch_vertex().vid();

    // record the vids that will be modified for roll backs on failure
    std::array<std::pair<size_t, VertexConnectivity>, 4> old_vertices;
    old_vertices[0] = {vid[0], m_vertex_connectivity[vid[0]]};
    old_vertices[1] = {vid[1], m_vertex_connectivity[vid[1]]};
    old_vertices[2] = {vid[2], m_vertex_connectivity[vid[2]]};
    old_vertices[3] = {vid[2], m_vertex_connectivity[vid[3]]};
    std::pair<size_t, TetrahedronConnectivity> old_tet;
    old_tet = std::make_pair(tid, m_tet_connectivity[tid]);

    const auto conn_tets = [this](size_t i) -> std::vector<size_t>& {
        return m_vertex_connectivity[i].m_conn_tets;
    };

    // update vertex connectivity
    const size_t new_vid = get_next_empty_slot_v();
    const size_t new_tid1 = get_next_empty_slot_t();
    const size_t new_tid2 = get_next_empty_slot_t();
    const size_t new_tid3 = get_next_empty_slot_t();

    vector_erase(conn_tets(vid[0]), tid);
    conn_tets(vid[0]).emplace_back(new_tid1);
    conn_tets(vid[0]).emplace_back(new_tid2);
    conn_tets(vid[0]).emplace_back(new_tid3);
    std::sort(conn_tets(vid[0]).begin(), conn_tets(vid[0]).end());

    conn_tets(vid[1]).emplace_back(new_tid2);
    conn_tets(vid[1]).emplace_back(new_tid3);
    std::sort(conn_tets(vid[1]).begin(), conn_tets(vid[1]).end());
    conn_tets(vid[2]).emplace_back(new_tid1);
    conn_tets(vid[2]).emplace_back(new_tid3);
    std::sort(conn_tets(vid[2]).begin(), conn_tets(vid[2]).end());
    conn_tets(vid[3]).emplace_back(new_tid1);
    conn_tets(vid[3]).emplace_back(new_tid2);
    std::sort(conn_tets(vid[3]).begin(), conn_tets(vid[3]).end());


    conn_tets(new_vid).reserve(4);
    conn_tets(new_vid).emplace_back(tid);
    conn_tets(new_vid).emplace_back(new_tid1);
    conn_tets(new_vid).emplace_back(new_tid2);
    conn_tets(new_vid).emplace_back(new_tid3);
    std::sort(conn_tets(new_vid).begin(), conn_tets(new_vid).end());

    // now the tets
    // need to update the hash of tid
    const size_t i = m_tet_connectivity[tid].find(vid[0]);
    const size_t j = m_tet_connectivity[tid].find(vid[1]);
    const size_t k = m_tet_connectivity[tid].find(vid[2]);
    const size_t l = m_tet_connectivity[tid].find(vid[3]);
    m_tet_connectivity[tid].m_indices[i] = new_vid;
    m_tet_connectivity[tid].hash++;
    // new_tid1/2/3 m_indices in same order
    m_tet_connectivity[new_tid1].m_indices[i] = vid[0];
    m_tet_connectivity[new_tid1].m_indices[j] = new_vid;
    m_tet_connectivity[new_tid1].m_indices[k] = vid[2];
    m_tet_connectivity[new_tid1].m_indices[l] = vid[3];
    m_tet_connectivity[new_tid2].m_indices[i] = vid[0];
    m_tet_connectivity[new_tid2].m_indices[j] = vid[1];
    m_tet_connectivity[new_tid2].m_indices[k] = new_vid;
    m_tet_connectivity[new_tid2].m_indices[l] = vid[3];

    m_tet_connectivity[new_tid3].m_indices[i] = vid[0];
    m_tet_connectivity[new_tid3].m_indices[j] = vid[1];
    m_tet_connectivity[new_tid3].m_indices[k] = vid[2];
    m_tet_connectivity[new_tid3].m_indices[l] = new_vid;

    // make the new tuple
    const size_t tid_for_return = new_tid3;
    const size_t eid_for_return =
        m_tet_connectivity[tid_for_return].find_local_edge(vid[0], new_vid);
    assert(eid_for_return != -1);
    const size_t fid_for_return =
        m_tet_connectivity[tid_for_return].find_local_face(vid[0], vid[1], new_vid);
    assert(fid_for_return != -1);

    const Tuple new_t(*this, vid[0], eid_for_return, fid_for_return, tid_for_return);

    new_tets = get_one_ring_tets_for_vertex(new_t.switch_vertex(*this));

    start_protect_attributes();
    if (!split_face_after(new_t) || !invariants(new_tets)) {
        // rollback topo
        // restore old v, t
        for (const auto& old_v : old_vertices) {
            m_vertex_connectivity[old_v.first] = old_v.second;
        }
        m_tet_connectivity[old_tet.first] = old_tet.second;

        // erase new_vid new_fids
        m_vertex_connectivity[new_vid].m_conn_tets.clear();
        m_vertex_connectivity[new_vid].m_is_removed = true;
        m_tet_connectivity[new_tid1].m_is_removed = true;
        m_tet_connectivity[new_tid2].m_is_removed = true;
        m_tet_connectivity[new_tid3].m_is_removed = true;
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();

    return true;
}

} // namespace wmtk