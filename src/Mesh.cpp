//
// Created by Yixin Hu on 11/3/21.
//

#include "Mesh.h"

void wmtk::TetMesh::split_edge(const Tuple &loc0) {
    if (!split_before(loc0))
        return;

    // backup of everything
    auto loc = loc0;
    int v1_id = loc.vid;
    loc.flip_vertex(*this);
    int v2_id = loc.vid;
    //
    auto n12_t_ids = set_intersection(m_vertex_connectivity[v1_id].m_conn_tets,
                                      m_vertex_connectivity[v2_id].m_conn_tets);
    std::vector<int> n12_v_ids;
    for(int t_id: n12_t_ids){
        for(int j=0;j<4;j++)
            n12_v_ids.push_back(m_tetrahedron_connectivity[t_id][j]);
    }
    vector_unique(n12_v_ids);
    std::vector<std::pair<int, TetrahedronConnectivity>> old_tets(n12_t_ids.size());
    std::vector<std::pair<int, VertexConnectivity>> old_vertices(n12_v_ids.size());
    for(int t_id: n12_t_ids)
        old_tets.push_back(std::make_pair(t_id, m_tetrahedron_connectivity[t_id]));
    for(int v_id: n12_v_ids)
        old_vertices.push_back(std::make_pair(v_id, m_vertex_connectivity[v_id]));

    // update connectivity
    int v_id = find_next_empty_slot_v();
    for(int t_id: n12_t_ids){
        int new_t_id = find_next_empty_slot_t();
        int j = m_tetrahedron_connectivity[t_id].find(v_id);
        m_tetrahedron_connectivity[new_t_id] = m_tetrahedron_connectivity[t_id];
        m_tetrahedron_connectivity[new_t_id][j] = v_id;
        //
        j = m_tetrahedron_connectivity[t_id].find(v2_id);
        m_tetrahedron_connectivity[t_id][j] = v_id;
        //
        //todo: update conn_tets for vertices
    }

    // possibly call the resize_attributes
    //todo

    if (!split_after(loc0))//todo: new mesh old loc???
    {
        // undo changes
        return;
    }

    // call invariants on all entities
    if (false) // if any invariant fails
    {
        // undo changes
        return;
    }

}
