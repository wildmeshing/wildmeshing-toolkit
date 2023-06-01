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
TriMesh::ProtectedAttributeRAII TriMeshOperation::start_protected_attributes_raii(TriMesh& m)
{
    return m.start_protected_attributes_raii();
}
TriMesh::ProtectedConnectivityRAII TriMeshOperation::start_protected_connectivity_raii(TriMesh& m)
{
    return m.start_protected_connectivity_raii();
}

void TriMeshOperation::rollback_protected(TriMesh& m) {
    m.rollback_protected();
}
    bool TriMeshOperation::invariants(TriMesh& m) {
        return m.invariants(*this);
    }


bool TriMeshOperation::operator()(TriMesh& m, const Tuple& t)
{
    auto attr_raa = start_protected_attributes_raii(m);
    auto con_raa = start_protected_connectivity_raii(m);

    if (before(m, t)) {
        if (execute(m, t)) { // success should be marked here
            if (after(m)) {
                if (invariants(m)) {
                    return true;
                }
            }
        }
    }
    m.rollback_protected();
    return false;
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

