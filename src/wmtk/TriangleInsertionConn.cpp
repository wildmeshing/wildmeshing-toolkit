//
// Created by Yixin Hu on 1/4/22.
//

#include <wmtk/TetMesh.h>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/auto_table.hpp>


void wmtk::TetMesh::subdivide_tets(const std::vector<size_t> intersected_tids, std::map<std::array<size_t, 2>, size_t>& map_edge2vid)
{
    /// insert new vertices
    size_t old_v_size = m_vertex_connectivity.size();
    m_vertex_connectivity.resize(m_vertex_connectivity.size() + map_edge2vid.size());

    /// insert new tets
    size_t old_t_size = m_tet_connectivity.size();
    for (size_t t_id : intersected_tids) {
        std::array<int, 6> new_v_ids = {{-1, -1, -1, -1, -1, -1}};
        for (int j = 0; j < 6; j++) {
            std::array<size_t, 2> e = {
                {m_tet_connectivity[t_id][m_local_edges[j][0]],
                 m_tet_connectivity[t_id][m_local_edges[j][1]]}};
            if (e[0] > e[1]) std::swap(e[0], e[1]);
            if (map_edge2vid.count(e)) new_v_ids[j] = map_edge2vid[e];
        }
        subdivide_a_tet(t_id, new_v_ids);
    }

    /// update conn_tets
    {
        std::vector<size_t> vids;
        for (size_t t_id : intersected_tids) {
            for (int j = 0; j < 4; j++) vids.push_back(m_tet_connectivity[t_id][j]);
        }
        vector_unique(vids);
        //
        std::vector<size_t> tids;
        for (size_t vid : vids) {
            tids.insert(
                tids.end(),
                m_vertex_connectivity[vid].m_conn_tets.begin(),
                m_vertex_connectivity[vid].m_conn_tets.end());
        }
        vector_unique(tids);
        //
        for (size_t i = old_t_size; i < m_tet_connectivity.size(); i++) tids.push_back(i);
        //
        for (size_t i = old_v_size; i < m_vertex_connectivity.size(); i++) vids.push_back(i);
        //
        std::map<size_t, std::vector<size_t>> new_conn_tets;
        for (size_t tid : tids) {
            for (int j = 0; j < 4; j++) {
                size_t vid = m_tet_connectivity[tid][j];
                if (new_conn_tets.count(vid)) {
                    new_conn_tets[vid].push_back(tid);
                } else {
                    new_conn_tets[vid] = {tid};
                }
            }
        }
        //
        for (int vid : vids) {
            m_vertex_connectivity[vid].m_conn_tets = new_conn_tets[vid];
            std::sort(
                m_vertex_connectivity[vid].m_conn_tets.begin(),
                m_vertex_connectivity[vid].m_conn_tets.end());
        }
    }
}

void wmtk::TetMesh::subdivide_a_tet(size_t t_id, const std::array<int, 6>& new_v_ids)
{
    std::map<std::array<int, 2>, int> get_local_e_id;
    for (int i = 0; i < m_local_edges.size(); i++) {
        get_local_e_id[m_local_edges[i]] = i;
        get_local_e_id[{{m_local_edges[i][1], m_local_edges[i][0]}}] = i;
    }

        std::vector<size_t> all_v_ids = {
            m_tet_connectivity[t_id][0],
            m_tet_connectivity[t_id][1],
            m_tet_connectivity[t_id][2],
            m_tet_connectivity[t_id][3]};
    std::array<int, 6> local_new_v_ids = {{-1, -1, -1, -1, -1, -1}};

    std::bitset<6> config_bits;
    int cnt = 0;
    for (int i = 0; i < new_v_ids.size(); i++) {
        if (new_v_ids[i] >= 0) {
                        all_v_ids.push_back(new_v_ids[i]);
            config_bits.set(i);
            local_new_v_ids[i] = 4 + cnt;
            cnt++;
        }
    }

    int config_id = (int)(config_bits.to_ulong());
    const auto configs = floatTetWild::CutTable::get_tet_confs(config_id);
    assert(!configs.empty());
    // note: config with centroid only happens on open boundary preservation

    std::vector<floatTetWild::Vector2i> my_diags;
    for (int j = 0; j < 4; j++) { // 4 faces of the tet
        int cnt_diags = 0;
        floatTetWild::Vector2i diag(-1, -1);
        for (int k = 0; k < 3; k++) {
            int l_eid = get_local_e_id[{{m_local_faces[j][k], m_local_faces[j][(k + 1) % 3]}}];
            if (new_v_ids[l_eid] >= 0) {
                if (new_v_ids[l_eid] > diag[1])
                    diag << m_local_faces[j][(k + 2) % 3], local_new_v_ids[l_eid];
                cnt_diags++;
            }
        }
        if (cnt_diags < 2) continue;

        if (diag[0] > diag[1]) std::swap(diag[0], diag[1]);
        my_diags.push_back(diag);
    }
    std::sort(
        my_diags.begin(),
        my_diags.end(),
        [](const floatTetWild::Vector2i& a, const floatTetWild::Vector2i& b) {
            return std::make_tuple(a[0], a[1]) < std::make_tuple(b[0], b[1]);
        });

    int diag_config_id = 0;
    if (!my_diags.empty()) {
        const auto& all_diags = floatTetWild::CutTable::get_diag_confs(config_id);
        for (int i = 0; i < all_diags.size(); i++) {
            if (my_diags != all_diags[i]) continue;
            diag_config_id = i;
            break;
        }
    }

    const auto& config = floatTetWild::CutTable::get_tet_conf(config_id, diag_config_id);
    const auto& new_is_surface_fs =
        floatTetWild::CutTable::get_surface_conf(config_id, diag_config_id);
    const auto& new_local_f_ids =
        floatTetWild::CutTable::get_face_id_conf(config_id, diag_config_id);
    for (int i = 0; i < config.size(); i++) {
        const auto& t = config[i];
        TetrahedronConnectivity tet;
        for (int j = 0; j < 4; j++) {
            tet[j] = all_v_ids[t[j]];
        }
        m_tet_connectivity.push_back(tet);

        //todo: track surface
    }
}
