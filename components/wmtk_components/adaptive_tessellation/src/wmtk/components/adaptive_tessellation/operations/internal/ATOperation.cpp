#include "ATOperation.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
namespace wmtk::components::adaptive_tessellation::operations::internal {
using namespace wmtk;
ATOperation::ATOperation(
    Mesh& uv_mesh,
    Mesh& position_mesh,
    std::vector<std::shared_ptr<Mesh>> edge_mesh_ptrs,
    std::map<Mesh*, Mesh*> sibling_meshes_map,
    MeshAttributeHandle<double>& uv_handle)
    : m_uv_mesh_ptr(std::make_shared<TriMesh>(static_cast<TriMesh&>(uv_mesh)))
    , m_position_mesh_ptr(std::make_shared<TriMesh>(static_cast<TriMesh&>(position_mesh)))
    , m_edge_mesh_ptrs(edge_mesh_ptrs)
    , m_sibling_meshes_map(sibling_meshes_map)
    , m_uv_handle(uv_handle)
{}

void ATOperation::initialize_invariants(const Mesh& input_mesh, const Mesh& uv_mesh)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(input_mesh);
    invariants.add(std::make_shared<TriangleInversionInvariant>(uv_mesh, m_uv_handle));
}
const std::shared_ptr<TriMesh> ATOperation::uv_mesh_ptr() const
{
    return m_uv_mesh_ptr;
}
const std::shared_ptr<TriMesh> ATOperation::position_mesh_ptr() const
{
    return m_position_mesh_ptr;
}
Mesh* ATOperation::sibling_edge_mesh_raw_ptr(Mesh* my_edge_mesh_ptr)
{
    return m_sibling_meshes_map[my_edge_mesh_ptr];
}
const std::shared_ptr<Mesh> ATOperation::edge_mesh_ptr(long i) const
{
    return m_edge_mesh_ptrs[i];
}

Simplex ATOperation::sibling_edge(Mesh* my_edge_mesh_ptr, const Simplex& s)
{
    assert(s.primitive_type() == PrimitiveType::Edge);
    Mesh* sibling_mesh_ptr = sibling_edge_mesh_raw_ptr(my_edge_mesh_ptr);
    std::vector<Simplex> sibling_edge = my_edge_mesh_ptr->map((*sibling_mesh_ptr), s);
    assert(sibling_edge.size() == 1);
    return sibling_edge[0];
}
} // namespace wmtk::components::adaptive_tessellation::operations::internal