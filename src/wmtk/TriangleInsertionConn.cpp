//
// Created by Yixin Hu on 1/4/22.
//

#include <wmtk/TetMesh.h>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/auto_table.hpp>


void wmtk::TetMesh::subdivide_tets(std::vector<size_t> t_ids, std::map<std::array<size_t, 2>, size_t>& map_edge2vid)
{
    m_vertex_connectivity.resize(m_vertex_connectivity.size() + map_edge2vid.size());

    for (size_t t_id : t_ids) {
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

    // todo: update conn_tets
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
