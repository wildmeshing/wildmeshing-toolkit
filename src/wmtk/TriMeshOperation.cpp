#include <wmtk/TriMeshOperation.h>
#include <wmtk/utils/VectorUtils.h>
using namespace wmtk;


auto TriMeshOperation::vertex_connectivity(TriMesh& m)
    -> wmtk::AttributeCollection<VertexConnectivity>&
{
    return m.m_vertex_connectivity;
}
auto TriMeshOperation::tri_connectivity(TriMesh& m)
    -> wmtk::AttributeCollection<TriangleConnectivity>&
{
    return m.m_tri_connectivity;
}
auto TriMeshOperation::vertex_connectivity(const TriMesh& m)
    -> const wmtk::AttributeCollection<VertexConnectivity>&
{
    return m.m_vertex_connectivity;
}
auto TriMeshOperation::tri_connectivity(const TriMesh& m)
    -> const wmtk::AttributeCollection<TriangleConnectivity>&
{
    return m.m_tri_connectivity;
}
size_t TriMeshOperation::get_next_empty_slot_t(TriMesh& m)
{
    return m.get_next_empty_slot_t();
}
size_t TriMeshOperation::get_next_empty_slot_v(TriMesh& m)
{
    return m.get_next_empty_slot_v();
}


auto TriMeshOperation::operator()(TriMesh& m, const Tuple& t) -> ExecuteReturnData
{
    ExecuteReturnData retdata;
    retdata.success = false;

    mark_failed();
    m.start_protected_connectivity();
    m.start_protected_attributes();

    retdata.success = before(m, t);
    if (!retdata.success) {
        goto finish;
    }

    retdata = execute(m, t);
    assign(retdata.tuple);
    if (!retdata.success) {
        goto finish;
    }

    retdata.success = after(m, retdata) && invariants(m, retdata);
    if (!retdata.success) {
        goto finish;
    }

    //
finish:
    if (!retdata.success) {
        m.rollback_protected();
        mark_failed();
    }
    m.release_protected_connectivity();
    m.release_protected_attributes();

    return retdata;
}

bool TriMeshOperation::invariants(TriMesh& m, ExecuteReturnData& ret_data)
{
    return m.invariants(ret_data.new_tris);
}

void TriMeshOperation::set_vertex_size(TriMesh& m, size_t v_cnt)
{
    m.current_vert_size = v_cnt;
    auto& vertex_con = vertex_connectivity(m);
    vertex_con.m_attributes.grow_to_at_least(v_cnt);
    vertex_con.shrink_to_fit();
    m.resize_mutex(m.vert_capacity());
}
void TriMeshOperation::set_tri_size(TriMesh& m, size_t t_cnt)
{
    m.current_tri_size = t_cnt;

    auto& tri_con = tri_connectivity(m);
    tri_con.m_attributes.grow_to_at_least(t_cnt);
    tri_con.shrink_to_fit();
}






