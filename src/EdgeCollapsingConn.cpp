//
// Created by Yixin Hu on 12/9/21.
//
#include <wmtk/TetMesh.h>

#include <wmtk/TupleUtils.hpp>

bool wmtk::TetMesh::collapse_edge(const Tuple& loc0, std::vector<Tuple>& new_edges)
{
    if (!collapse_before(loc0)) return false;

    /// backup of everything
    auto loc1 = loc0;
    int v1_id = loc1.vid();
    auto loc2 = switch_vertex(loc1);
    int v2_id = loc2.vid();
//    std::cout << v1_id << " " << v2_id << std::endl;
    logger().trace("{} {}", v1_id, v2_id);
    //    loc1.print_info();
    //    loc2.print_info();
    //

    std::vector<size_t> n1_v_ids;
    for (size_t t_id : m_vertex_connectivity[v1_id].m_conn_tets) {
        for (int j = 0; j < 4; j++) n1_v_ids.push_back(m_tet_connectivity[t_id][j]);
    }
    vector_unique(n1_v_ids);
    std::vector<std::pair<size_t, TetrahedronConnectivity>> old_tets;
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices;
    for (size_t t_id : m_vertex_connectivity[v1_id].m_conn_tets)
        old_tets.push_back(std::make_pair(t_id, m_tet_connectivity[t_id]));
    for (size_t v_id : n1_v_ids)
        old_vertices.push_back(std::make_pair(v_id, m_vertex_connectivity[v_id]));

//    //fortest
//    auto old_m_tet_connectivity = m_tet_connectivity;
//    auto old_m_vertex_connectivity = m_vertex_connectivity;
//    //fortest

    /// update connectivity
    auto n1_t_ids =
        m_vertex_connectivity[v1_id].m_conn_tets; // note: conn_tets for v1 without removed tets
    m_vertex_connectivity[v1_id].m_is_removed = true;
    auto n12_t_ids = set_intersection(
        m_vertex_connectivity[v1_id].m_conn_tets,
        m_vertex_connectivity[v2_id].m_conn_tets);
    for (size_t t_id : n12_t_ids) {
        m_tet_connectivity[t_id].m_is_removed = true;
        vector_erase(m_vertex_connectivity[v2_id].m_conn_tets, t_id);
        vector_erase(n1_t_ids, t_id); // to add to conn_tets for v2
        //
        for (int j = 0; j < 4; j++) {
            int v_id = m_tet_connectivity[t_id][j];
            if (v_id != v1_id && v_id != v2_id)
                vector_erase(m_vertex_connectivity[v_id].m_conn_tets, t_id);
        }
    }
    //
    for (size_t t_id : n1_t_ids) {
        m_vertex_connectivity[v2_id].m_conn_tets.push_back(t_id);
        int j = m_tet_connectivity[t_id].find(v1_id);
        assert(j>=0);
        m_tet_connectivity[t_id][j] = v2_id;
    }
    vector_sort(m_vertex_connectivity[v2_id].m_conn_tets);
    //
    m_vertex_connectivity[v1_id].m_conn_tets.clear(); // release mem

    Tuple new_loc = tuple_from_vertex(v2_id);
    // todo: update timestamp of tets and tuple, change locs to loc
    if (!collapse_after(new_loc)) {
        m_vertex_connectivity[v1_id].m_is_removed = false;
        for (int t_id : n12_t_ids) m_tet_connectivity[t_id].m_is_removed = false;
        //
        for (int i = 0; i < old_tets.size(); i++) {
            int t_id = old_tets[i].first;
            m_tet_connectivity[t_id] = old_tets[i].second;
        }
        for (int i = 0; i < old_vertices.size(); i++) {
            int v_id = old_vertices[i].first;
            m_vertex_connectivity[v_id] = old_vertices[i].second;
        }

//        //fortest
//        std::cout<<old_m_vertex_connectivity.size()<<" "<<m_vertex_connectivity.size()<<std::endl;
//        for(int i=0;i<old_m_vertex_connectivity.size();i++){
//            if(!(old_m_vertex_connectivity[i] == m_vertex_connectivity[i])){
//                std::cout<<i<<std::endl;
//                old_m_vertex_connectivity[i].print_info();
//                std::cout<<"//"<<std::endl;
//                m_vertex_connectivity[i].print_info();
//            }
//        }
//
//        std::cout<<old_m_tet_connectivity.size()<<" "<<m_tet_connectivity.size()<<std::endl;
//        for(int i=0;i<old_m_tet_connectivity.size();i++){
//            if(!(old_m_tet_connectivity[i] == m_tet_connectivity[i])){
//                std::cout<<i<<std::endl;
//                old_m_tet_connectivity[i].print_info();
//                std::cout<<"//"<<std::endl;
//                m_tet_connectivity[i].print_info();
//            }
//        }
//
//        assert(old_m_vertex_connectivity == m_vertex_connectivity);
//        assert(old_m_tet_connectivity == m_tet_connectivity);
//        //fortest

        return false;
    }

    /// call invariants on all entities
    if (false) // if any invariant fails
    {
        m_vertex_connectivity[v1_id].m_is_removed = false;
        for (int t_id : n12_t_ids) m_tet_connectivity[t_id].m_is_removed = false;
        //
        for (int i = 0; i < old_tets.size(); i++) {
            int t_id = old_tets[i].first;
            m_tet_connectivity[t_id] = old_tets[i].second;
        }
        for (int i = 0; i < old_vertices.size(); i++) {
            int v_id = old_vertices[i].first;
            m_vertex_connectivity[v_id] = old_vertices[i].second;
        }

        return false;
    }

    /// return new_edges
    for (size_t t_id : n1_t_ids) {
        for (int j = 0; j < 6; j++) {
            if (m_tet_connectivity[t_id][m_local_edges[j][0]] == v2_id ||
                m_tet_connectivity[t_id][m_local_edges[j][1]] == v2_id)
                new_edges.push_back(tuple_from_edge(t_id, j));
        }
    }
    unique_edge_tuples(*this, new_edges);

    /// update timestamps
//    m_timestamp++; // todo: thread
//    for (size_t t_id : n1_t_ids) m_tet_connectivity[t_id].set_version_number(m_timestamp);
//    for (auto& new_loc : new_edges) // update edge timestamp from tets
//        new_loc.update_version_number(*this);


    return true;
}