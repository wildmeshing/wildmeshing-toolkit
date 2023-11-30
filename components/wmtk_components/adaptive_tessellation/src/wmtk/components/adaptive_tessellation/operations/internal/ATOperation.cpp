#include "ATOperation.hpp"
#include <wmtk/Primitive.hpp>
namespace wmtk::components::adaptive_tessellation::operations::internal {
using namespace wmtk;
ATOperation::ATOperation(
    TriMesh& uv_mesh,
    TriMesh& position_mesh,
    std::vector<std::shared_ptr<EdgeMesh>> edge_mesh_ptrs,
    std::map<Mesh*, Mesh*> sibling_meshes_map)
    : m_uv_mesh(uv_mesh)
    , m_position_mesh(position_mesh)
    , m_edge_mesh_ptrs(edge_mesh_ptrs)
    , m_sibling_meshes_map(sibling_meshes_map)
{}

void ATOperation::initialize_invariants(const Mesh& input_m, const TriMesh& uv_m) {}
const TriMesh& ATOperation::uv_mesh() const
{
    return m_uv_mesh;
}
const TriMesh& ATOperation::position_mesh() const
{
    return m_position_mesh;
}
Mesh* ATOperation::sibling_edge_mesh_ptr(Mesh* my_edge_mesh_ptr)
{
    return m_sibling_meshes_map[my_edge_mesh_ptr];
}
const EdgeMesh& ATOperation::edge_mesh(long i) const
{
    return *m_edge_mesh_ptrs[i].get();
}

Simplex ATOperation::sibling_edge(Mesh* my_edge_mesh_ptr, const Simplex& s)
{
    assert(s.primitive_type() == PrimitiveType::Edge);
    Mesh* sibling_mesh_ptr = sibling_edge_mesh_ptr(my_edge_mesh_ptr);
    std::vector<Simplex> sibling_edge = my_edge_mesh_ptr->map((*sibling_mesh_ptr), s);
    assert(sibling_edge.size() == 1);
    return sibling_edge[0];
}
} // namespace wmtk::components::adaptive_tessellation::operations::internal