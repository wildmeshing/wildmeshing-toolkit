//
// Created by Yixin Hu on 1/4/22.
//

#include <wmtk/TetMesh.h>
#include <wmtk/auto_table.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <bitset>

//fortest
using std::cout;
using std::endl;
//fortest

void wmtk::TetMesh::subdivide_tets(
    const std::vector<size_t> intersected_tids,
    std::map<std::array<size_t, 2>, size_t>& map_edge2vid)
{
    using namespace Eigen;
    /// insert new vertices
    size_t old_v_size = m_vertex_connectivity.size();
    m_vertex_connectivity.resize(m_vertex_connectivity.size() + map_edge2vid.size());


    /// record infos
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
        bool is_add_centroid;//todo: maybe not necessary
        subdivide_a_tet(t_id, new_v_ids, is_add_centroid);
//        if(is_add_centroid)
//            cout<<"is_add_centroid"<<endl;
    }

//    for(auto& info: map_edge2vid){
//        cout<<info.second<<endl;
//    }

    /// update conn_tets
    {
        for (size_t i = old_t_size; i < m_tet_connectivity.size(); i++) tids.push_back(i);
        //
        for (size_t i = old_v_size; i < m_vertex_connectivity.size(); i++) vids.push_back(i);


        std::map<size_t, std::vector<size_t>> new_conn_tets;
        for (int vid : vids) {
            new_conn_tets[vid] = {};
//            cout<<"v"<<vid<<endl;
        }
        //
        for (size_t tid : tids) {
            for (int j = 0; j < 4; j++) {
                size_t vid = m_tet_connectivity[tid][j];
                if (new_conn_tets.count(vid)) {
                    new_conn_tets[vid].push_back(tid);
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

void wmtk::TetMesh::subdivide_a_tet(size_t t_id, const std::array<int, 6>& new_v_ids, bool is_add_centroid)
{
    using namespace Eigen;

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
    const auto configs = CutTable::get_tet_confs(config_id);
    assert(!configs.empty());
    // note: config with centroid only happens on open boundary preservation

    std::vector<Vector2i> my_diags;
    for (int j = 0; j < 4; j++) { // 4 faces of the tet
        int cnt_diags = 0;
        Vector2i diag(-1, -1);
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
    std::sort(my_diags.begin(), my_diags.end(), [](const Vector2i& a, const Vector2i& b) {
        return std::make_tuple(a[0], a[1]) < std::make_tuple(b[0], b[1]);
    });

    int diag_config_id = -1;
    if (!my_diags.empty()) {
        const auto& all_diags = CutTable::get_diag_confs(config_id);
        for (int i = 0; i < all_diags.size(); i++) {
            if (my_diags != all_diags[i]) continue;
            diag_config_id = i;
            break;
        }
    } else
        diag_config_id = 0;
    assert(diag_config_id >= 0);

    // note: in some cases, we have to add centroid since the current setting of diags enforces that

    const auto& config = CutTable::get_tet_conf(config_id, diag_config_id);
    const auto& new_is_surface_fs = CutTable::get_surface_conf(config_id, diag_config_id);
    const auto& new_local_f_ids = CutTable::get_face_id_conf(config_id, diag_config_id);
//        if(config.size()==10){
//            cout<<"config.size() "<<config.size()<<endl;
//            cout<<config_id<<endl;
//            cout<<diag_config_id<<endl;
//            for(int j=0;j<6;j++)
//                cout<<new_v_ids[j]<<" ";
//            cout<<endl;
//        }
    is_add_centroid = false;
//    cout<<"t_id "<<t_id<<endl;
//    cout<<"config_id "<<config_id<<endl;
//    cout<<"config.size() "<<config.size()<<endl;
    for (int i = 0; i < config.size(); i++) {
        const auto& t = config[i];
        TetrahedronConnectivity tet;
        for (int j = 0; j < 4; j++) {
            if (!is_add_centroid && t[j] >= 4 + config_bits.count()) {
                add_tet_centroid(
                    {{m_tet_connectivity[t_id][0],
                      m_tet_connectivity[t_id][1],
                      m_tet_connectivity[t_id][2],
                      m_tet_connectivity[t_id][3]}});
                m_vertex_connectivity.emplace_back();
                all_v_ids.push_back(m_vertex_connectivity.size()-1);
//                cout<<"m_vertex_connectivity.size()-1) "<<m_vertex_connectivity.size()-1<<endl;
                is_add_centroid = true;
            }
            tet[j] = all_v_ids[t[j]];
        }
        size_t new_t_id = t_id;
        if (i < config.size() - 1) {
            m_tet_connectivity.emplace_back();
            new_t_id = m_tet_connectivity.size() - 1;
        }
        m_tet_connectivity[new_t_id] = tet;

//                cout<<"t"<<new_t_id<<": "<<tet[0]<<" "<<tet[1]<<" "<<tet[2]<<" "<<tet[3]<<endl;
//                cout<<"t"<<new_t_id<<": "<<t[0]<<" "<<t[1]<<" "<<t[2]<<" "<<t[3]<<endl;

        insertion_update_surface_tag(t_id, new_t_id, config_id, diag_config_id, i);
    }
}
