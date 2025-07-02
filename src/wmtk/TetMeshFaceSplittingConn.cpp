#include <wmtk/TetMesh.h>


namespace wmtk {

bool TetMesh::split_face(const Tuple& t, std::vector<Tuple>& new_tets)
{
    if (!split_face_before(t)) {
        return false;
    }
    if (!t.is_valid(*this)) {
        return false;
    }

    const std::optional<Tuple> t_opp = t.switch_tetrahedron(*this);

    /**
     *
     *
     *         v2
     *         /|\
     *        / | \
     *       /  |  \
     *      /t1 ^ t0\
     *     /  /   \  \
     *    //   t2    \\
     *  v0 ----------- v1
     *
     */
    const size_t tid = t.tid(*this);
    const size_t vid_link = t.switch_face(*this).switch_edge(*this).switch_vertex(*this).vid(*this);


    std::optional<size_t> tid_opp;
    std::optional<size_t> vid_link_opp;
    if (t_opp) {
        tid_opp = t_opp.value().tid(*this);
        vid_link_opp =
            t_opp.value().switch_face(*this).switch_edge(*this).switch_vertex(*this).vid(*this);
    }

    std::array<size_t, 3> vid;
    vid[0] = t.vid(*this);
    vid[1] = t.switch_vertex(*this).vid(*this);
    vid[2] = t.switch_edge(*this).switch_vertex(*this).vid(*this);

    // record the vids that will be modified for roll backs on failure
    std::array<std::pair<size_t, VertexConnectivity>, 3> old_vertices;
    old_vertices[0] = {vid[0], m_vertex_connectivity[vid[0]]};
    old_vertices[1] = {vid[1], m_vertex_connectivity[vid[1]]};
    old_vertices[2] = {vid[2], m_vertex_connectivity[vid[2]]};
    std::pair<size_t, TetrahedronConnectivity> old_tet;
    old_tet = std::make_pair(tid, m_tet_connectivity[tid]);
    std::optional<std::pair<size_t, TetrahedronConnectivity>> old_tet_opp;
    if (t_opp) {
        old_tet_opp = std::make_pair(tid_opp.value(), m_tet_connectivity[tid_opp.value()]);
    }

    const auto conn_tets = [this](size_t i) -> std::vector<size_t>& {
        return m_vertex_connectivity[i].m_conn_tets;
    };

    // update vertex connectivity
    const size_t new_vid = get_next_empty_slot_v();
    const size_t new_tid1 = get_next_empty_slot_t();
    const size_t new_tid2 = get_next_empty_slot_t();
    std::optional<size_t> new_tid1_opp;
    std::optional<size_t> new_tid2_opp;
    if (t_opp) {
        new_tid1_opp = get_next_empty_slot_t();
        new_tid2_opp = get_next_empty_slot_t();
    }

    vector_erase(conn_tets(vid[0]), tid);
    conn_tets(vid[0]).emplace_back(new_tid1);
    conn_tets(vid[0]).emplace_back(new_tid2);
    conn_tets(vid[1]).emplace_back(new_tid2);
    conn_tets(vid[2]).emplace_back(new_tid1);
    if (t_opp) {
        vector_erase(conn_tets(vid[0]), tid_opp.value());
        conn_tets(vid[0]).emplace_back(new_tid1_opp.value());
        conn_tets(vid[0]).emplace_back(new_tid2_opp.value());
        conn_tets(vid[1]).emplace_back(new_tid2_opp.value());
        conn_tets(vid[2]).emplace_back(new_tid1_opp.value());
    }
    std::sort(conn_tets(vid[0]).begin(), conn_tets(vid[0]).end());
    std::sort(conn_tets(vid[1]).begin(), conn_tets(vid[1]).end());
    std::sort(conn_tets(vid[2]).begin(), conn_tets(vid[2]).end());

    conn_tets(vid_link).emplace_back(new_tid1);
    conn_tets(vid_link).emplace_back(new_tid2);
    std::sort(conn_tets(vid_link).begin(), conn_tets(vid_link).end());
    if (t_opp) {
        conn_tets(vid_link_opp.value()).emplace_back(new_tid1_opp.value());
        conn_tets(vid_link_opp.value()).emplace_back(new_tid2_opp.value());
        std::sort(conn_tets(vid_link_opp.value()).begin(), conn_tets(vid_link_opp.value()).end());
    }

    conn_tets(new_vid).reserve(2 * 3);
    conn_tets(new_vid).emplace_back(tid);
    conn_tets(new_vid).emplace_back(new_tid1);
    conn_tets(new_vid).emplace_back(new_tid2);
    if (t_opp) {
        conn_tets(new_vid).emplace_back(tid_opp.value());
        conn_tets(new_vid).emplace_back(new_tid1_opp.value());
        conn_tets(new_vid).emplace_back(new_tid2_opp.value());
    }
    std::sort(conn_tets(new_vid).begin(), conn_tets(new_vid).end());

    // now the tets
    // need to update the hash of tid
    const size_t i = m_tet_connectivity[tid].find(vid[0]);
    const size_t j = m_tet_connectivity[tid].find(vid[1]);
    const size_t k = m_tet_connectivity[tid].find(vid[2]);
    const size_t l = m_tet_connectivity[tid].find(vid_link);
    m_tet_connectivity[tid].m_indices[i] = new_vid;
    m_tet_connectivity[tid].hash++;
    // new_tid1/2 m_indices in same order
    m_tet_connectivity[new_tid1].m_indices[i] = vid[0];
    m_tet_connectivity[new_tid1].m_indices[j] = new_vid;
    m_tet_connectivity[new_tid1].m_indices[k] = vid[2];
    m_tet_connectivity[new_tid1].m_indices[l] = vid_link;
    m_tet_connectivity[new_tid2].m_indices[i] = vid[0];
    m_tet_connectivity[new_tid2].m_indices[j] = vid[1];
    m_tet_connectivity[new_tid2].m_indices[k] = new_vid;
    m_tet_connectivity[new_tid2].m_indices[l] = vid_link;
    if (t_opp) {
        const size_t i_opp = m_tet_connectivity[tid_opp.value()].find(vid[0]);
        const size_t j_opp = m_tet_connectivity[tid_opp.value()].find(vid[1]);
        const size_t k_opp = m_tet_connectivity[tid_opp.value()].find(vid[2]);
        const size_t l_opp = m_tet_connectivity[tid_opp.value()].find(vid_link_opp.value());
        m_tet_connectivity[tid_opp.value()].m_indices[i_opp] = new_vid;
        m_tet_connectivity[tid_opp.value()].hash++;
        // new_tid1/2_opp m_indices in same order
        m_tet_connectivity[new_tid1_opp.value()].m_indices[i_opp] = vid[0];
        m_tet_connectivity[new_tid1_opp.value()].m_indices[j_opp] = new_vid;
        m_tet_connectivity[new_tid1_opp.value()].m_indices[k_opp] = vid[2];
        m_tet_connectivity[new_tid1_opp.value()].m_indices[l_opp] = vid_link_opp.value();
        m_tet_connectivity[new_tid2_opp.value()].m_indices[i_opp] = vid[0];
        m_tet_connectivity[new_tid2_opp.value()].m_indices[j_opp] = vid[1];
        m_tet_connectivity[new_tid2_opp.value()].m_indices[k_opp] = new_vid;
        m_tet_connectivity[new_tid2_opp.value()].m_indices[l_opp] = vid_link_opp.value();
    }

    // make the new tuple
    const size_t tid_for_return = new_tid2;
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
        if (t_opp) {
            m_tet_connectivity[old_tet_opp.value().first] = old_tet_opp.value().second;
        }

        // erase new_vid new_fids
        m_vertex_connectivity[new_vid].m_conn_tets.clear();
        m_vertex_connectivity[new_vid].m_is_removed = true;
        m_tet_connectivity[new_tid1].m_is_removed = true;
        m_tet_connectivity[new_tid2].m_is_removed = true;
        if (t_opp) {
            m_tet_connectivity[new_tid1_opp.value()].m_is_removed = true;
            m_tet_connectivity[new_tid2_opp.value()].m_is_removed = true;
        }
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();

    return true;
}

} // namespace wmtk