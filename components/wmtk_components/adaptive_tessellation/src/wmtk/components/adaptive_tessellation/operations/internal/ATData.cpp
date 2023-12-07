#include "ATData.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
namespace wmtk::components::adaptive_tessellation::operations::internal {
using namespace wmtk;
ATData::ATData(
    TriMesh& uv_mesh,
    TriMesh& position_mesh,
    std::vector<std::shared_ptr<EdgeMesh>> edge_mesh_ptrs,
    std::map<EdgeMesh*, EdgeMesh*> sibling_meshes_map,
    MeshAttributeHandle<double>& uv_handle,
    MeshAttributeHandle<double>& position_handle)
    : m_uv_mesh_ptr(uv_mesh.shared_from_this())
    , m_position_mesh_ptr(position_mesh.shared_from_this())
    , m_edge_mesh_ptrs(edge_mesh_ptrs)
    , m_sibling_meshes_map(sibling_meshes_map)
    , m_uv_handle(uv_handle)
    , m_position_handle(position_handle)
{}

void ATData::initialize_invariants(const Mesh& input_m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(input_m);
    invariants.add(std::make_shared<TriangleInversionInvariant>(*uv_mesh_ptr(), m_uv_handle));
}
const std::shared_ptr<TriMesh> ATData::uv_mesh_ptr() const
{
    return m_uv_mesh_ptr;
}
const std::shared_ptr<TriMesh> ATData::position_mesh_ptr() const
{
    return m_position_mesh_ptr;
}
EdgeMesh* ATData::sibling_edge_mesh_raw_ptr(EdgeMesh* my_edge_mesh_ptr)
{
    return m_sibling_meshes_map[my_edge_mesh_ptr];
}
const std::shared_ptr<EdgeMesh> ATData::edge_mesh_ptr(long i) const
{
    return m_edge_mesh_ptrs[i];
}

Simplex ATData::sibling_edge(EdgeMesh* my_edge_mesh_ptr, const Simplex& s)
{
    assert(s.primitive_type() == PrimitiveType::Edge);
    EdgeMesh* sibling_mesh_ptr = sibling_edge_mesh_raw_ptr(my_edge_mesh_ptr);
    std::vector<Simplex> sibling_edge = my_edge_mesh_ptr->map((*sibling_mesh_ptr), s);
    assert(sibling_edge.size() == 1);
    return sibling_edge[0];
}
} // namespace wmtk::components::adaptive_tessellation::operations::internal
