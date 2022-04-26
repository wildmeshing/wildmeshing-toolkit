#include <wmtk/TetMesh.h>

#include <algorithm>
#include <wmtk/utils/TupleUtils.hpp>

bool wmtk::TetMesh::split_edge(const Tuple& loc0, std::vector<Tuple>& new_edges)
{
    if (!split_edge_before(loc0)) return false;

    // backup of everything
    auto loc1 = loc0;
    int v1_id = loc1.vid(*this);
    auto loc2 = switch_vertex(loc1);
    int v2_id = loc2.vid(*this);
    logger().trace("{} {}", v1_id, v2_id);

    auto n12_t_ids = set_intersection(
        m_vertex_connectivity[v1_id].m_conn_tets,
        m_vertex_connectivity[v2_id].m_conn_tets);
    std::vector<size_t> n12_v_ids;
    for (size_t t_id : n12_t_ids) {
        for (int j = 0; j < 4; j++) n12_v_ids.push_back(m_tet_connectivity[t_id][j]);
    }
    vector_unique(n12_v_ids);
    std::vector<std::pair<size_t, TetrahedronConnectivity>> old_tets;
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices;
    for (size_t t_id : n12_t_ids) old_tets.emplace_back(t_id, m_tet_connectivity[t_id]);
    for (size_t v_id : n12_v_ids) old_vertices.emplace_back(v_id, m_vertex_connectivity[v_id]);

    /// update connectivity
    int v_id = get_next_empty_slot_v();
    std::vector<TetrahedronConnectivity> old_tets_conn;
    std::vector<std::array<size_t, 4>> new_tet_conn;
    auto num = n12_t_ids.size();
    new_tet_conn.resize(num * 2);
    for (auto i = 0; i < n12_t_ids.size(); i++) {
        auto t_id = n12_t_ids[i];
        auto& tet = m_tet_connectivity[t_id];
        old_tets_conn.push_back(tet);
        {
            auto l = tet.find(v2_id);
            new_tet_conn[i] = (tet.m_indices);
            new_tet_conn[i][l] = v_id;
        }
        {
            auto l = tet.find(v1_id);
            new_tet_conn[i + num] = (tet.m_indices);
            new_tet_conn[i + num][l] = v_id;
        }
    }

    auto new_tet_id = n12_t_ids;
    auto rollback_vert_conn = operation_update_connectivity_impl(new_tet_id, new_tet_conn);

    Tuple new_loc = tuple_from_vertex(v_id);

    start_protect_attributes();
    if (!split_edge_after(new_loc) || !invariants(get_one_ring_tets_for_vertex(new_loc))) {
        m_vertex_connectivity[v_id].m_is_removed = true;
        m_vertex_connectivity[v_id].m_conn_tets.clear();

        operation_failure_rollback_imp(rollback_vert_conn, n12_t_ids, new_tet_id, old_tets_conn);

        return false;
    }
    release_protect_attributes();

    // new_edges
    assert(std::is_sorted(new_tet_id.begin(), new_tet_id.end()));
    for (size_t t_id : new_tet_id) {
        for (int j = 0; j < 6; j++) {
            new_edges.push_back(tuple_from_edge(t_id, j));
        }
    }
    unique_edge_tuples(*this, new_edges);

    return true;
}