#include <wmtk/TetMesh.h>

#include <wmtk/utils/TupleUtils.hpp>

bool wmtk::TetMesh::split_edge(const Tuple& loc0, std::vector<Tuple>& new_edges)
{
    if (!split_before(loc0)) return false;

    // backup of everything
    auto loc1 = loc0;
    int v1_id = loc1.vid(*this);
    auto loc2 = switch_vertex(loc1);
    int v2_id = loc2.vid(*this);
    logger().trace("{} {}", v1_id, v2_id);

    auto n12_t_ids = set_intersection(
        m_vertex_connectivity[v1_id].m_conn_tets,
        m_vertex_connectivity[v2_id].m_conn_tets);

    /// update connectivity
    auto v_id = get_next_empty_slot_v();
    std::vector<size_t> new_t_ids;
    std::vector<TetrahedronConnectivity> old_tets;
    std::vector<std::array<size_t, 4>> new_tet_conn;
    for (size_t t_id : n12_t_ids) {
        auto& tet = m_tet_connectivity[t_id];
        old_tets.push_back(tet);
        for (auto vx : {v1_id, v2_id}) {
            auto l = tet.find(vx);
            new_tet_conn.push_back(tet.m_indices);
            new_tet_conn.back()[l] = v_id;
        }
    }

    auto new_tet_id = n12_t_ids;
    auto rollback_vert_conn = operation_update_connectivity_impl(new_tet_id, new_tet_conn);

    Tuple new_loc = tuple_from_vertex(v_id);
    if (!vertex_invariant(new_loc) || !edge_invariant(new_loc) || !tetrahedron_invariant(new_loc) ||
        !split_after(new_loc)) {
        m_vertex_connectivity[v_id].m_is_removed = true;
        m_vertex_connectivity[v_id].m_conn_tets.clear();
              operation_failure_rollback_imp(
            rollback_vert_conn,
            n12_t_ids,
            new_tet_id,
            old_tets);

        return false;
    }

    // new_edges
    for (size_t t_id : new_tet_id) {
        for (int j = 0; j < 6; j++) {
            new_edges.push_back(tuple_from_edge(t_id, j));
        }
    }
    unique_edge_tuples(*this, new_edges);

    return true;
}