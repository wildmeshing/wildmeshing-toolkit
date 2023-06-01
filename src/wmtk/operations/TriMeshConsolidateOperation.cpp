#include <wmtk/operations/TriMeshConsolidateOperation.h>
namespace wmtk {
bool TriMeshConsolidateOperation::execute(TriMesh& m, const Tuple& t)
{
    auto& vertex_con = vertex_connectivity(m);
    auto& tri_con = tri_connectivity(m);

    auto v_cnt = 0;
    std::vector<size_t> map_v_ids(m.vert_capacity(), -1);
    for (auto i = 0; i < m.vert_capacity(); i++) {
        if (vertex_con[i].m_is_removed) continue;
        map_v_ids[i] = v_cnt;
        v_cnt++;
    }
    auto t_cnt = 0;
    std::vector<size_t> map_t_ids(m.tri_capacity(), -1);
    for (auto i = 0; i < m.tri_capacity(); i++) {
        if (tri_con[i].m_is_removed) continue;
        map_t_ids[i] = t_cnt;
        t_cnt++;
    }
    v_cnt = 0;
    for (auto i = 0; i < m.vert_capacity(); i++) {
        if (vertex_con[i].m_is_removed) continue;
        if (v_cnt != i) {
            assert(v_cnt < i);
            vertex_con[v_cnt] = vertex_con[i];
            if (m.p_vertex_attrs) m.p_vertex_attrs->move(i, v_cnt);
        }
        for (size_t& t_id : vertex_con[v_cnt].m_conn_tris) {
            t_id = map_t_ids[t_id];
        }
        v_cnt++;
    }
    t_cnt = 0;
    for (int i = 0; i < m.tri_capacity(); i++) {
        if (tri_con[i].m_is_removed) continue;

        if (t_cnt != i) {
            assert(t_cnt < i);
            tri_con[t_cnt] = tri_con[i];
            tri_con[t_cnt].hash = 0;
            if (m.p_face_attrs) {
                m.p_face_attrs->move(i, t_cnt);
            }

            for (auto j = 0; j < 3; j++) {
                if (m.p_edge_attrs) {
                    m.p_edge_attrs->move(i * 3 + j, t_cnt * 3 + j);
                }
            }
        }
        for (size_t& v_id : tri_con[t_cnt].m_indices) {
            v_id = map_v_ids[v_id];
        }
        t_cnt++;
    }

    set_vertex_size(m, v_cnt);
    set_tri_size(m, t_cnt);

    // Resize user class attributes
    if (m.p_vertex_attrs) m.p_vertex_attrs->grow_to_at_least(m.vert_capacity());
    if (m.p_edge_attrs) m.p_edge_attrs->grow_to_at_least(m.tri_capacity() * 3);
    if (m.p_face_attrs) m.p_face_attrs->grow_to_at_least(m.tri_capacity());

    assert(m.check_edge_manifold());
    assert(m.check_mesh_connectivity_validity());
    return true;
}
bool TriMeshConsolidateOperation::before(TriMesh& m, const Tuple& t)
{
    return true;
}
bool TriMeshConsolidateOperation::after(TriMesh& m)
{
    return true;
}
std::string TriMeshConsolidateOperation::name() const
{
    return "consolidate";
}

// TODO: label triangles that actually moved
std::vector<TriMeshTuple> TriMeshConsolidateOperation::modified_triangles(const TriMesh& m) const
{
    return m.get_faces();
}
} // namespace wmtk
