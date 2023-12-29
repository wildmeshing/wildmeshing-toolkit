#include "ATData.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/utils/create_tag.hpp>
#include <wmtk/multimesh/utils/edge_meshes_parameterization.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/multimesh/utils/find_critical_points.hpp>
#include <wmtk/multimesh/utils/map_sibling_edge_meshes.hpp>

namespace ATmultimesh = wmtk::components::adaptive_tessellation::multimesh;
namespace wmtk::components::adaptive_tessellation::operations::internal {
using namespace wmtk;
ATData::ATData(
    std::shared_ptr<TriMesh> uv_mesh_ptr,
    std::shared_ptr<TriMesh> position_mesh_ptr,
    std::vector<std::shared_ptr<Mesh>> edge_mesh_ptrs,
    std::map<Mesh*, Mesh*> sibling_meshes_map,
    MeshAttributeHandle<double>& uv_handle)
    : m_uv_mesh_ptr(uv_mesh_ptr)
    , m_position_mesh_ptr(position_mesh_ptr)
    , m_edge_mesh_ptrs(edge_mesh_ptrs)
    , m_sibling_meshes_map(sibling_meshes_map)
    , m_uv_handle(uv_handle)
{}

ATData::ATData(
    std::shared_ptr<TriMesh> uv_mesh_ptr,
    std::shared_ptr<TriMesh> position_mesh_ptr,
    MeshAttributeHandle<double>& uv_handle)
    : m_uv_mesh_ptr(uv_mesh_ptr)
    , m_position_mesh_ptr(position_mesh_ptr)
    , m_uv_handle(uv_handle)
{
    auto uv_mesh_map =
        wmtk::multimesh::same_simplex_dimension_bijection(position_mesh(), uv_mesh());
    position_mesh().register_child_mesh(m_uv_mesh_ptr, uv_mesh_map);

    std::set<Tuple> critical_points =
        ATmultimesh::utils::find_critical_points(uv_mesh(), position_mesh());
    auto tags = wmtk::multimesh::utils::create_tags(uv_mesh(), critical_points);


    for (long tag : tags) {
        m_edge_mesh_ptrs.emplace_back(
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                uv_mesh(),
                "edge_tag",
                tag,
                PrimitiveType::Edge));
    }
    std::map<Mesh*, Mesh*> sibling_meshes_map =
        ATmultimesh::utils::map_sibling_edge_meshes(m_edge_mesh_ptrs);
    m_sibling_meshes_map = sibling_meshes_map;
    ATmultimesh::utils::parameterize_all_edge_meshes(
        uv_mesh(),
        m_edge_mesh_ptrs,
        m_sibling_meshes_map);
}


TriMesh& ATData::uv_mesh() const
{
    return *m_uv_mesh_ptr;
}

TriMesh& ATData::position_mesh() const
{
    return *m_position_mesh_ptr;
}
Mesh* ATData::sibling_edge_mesh_ptr(Mesh* my_edge_mesh_ptr)
{
    return m_sibling_meshes_map[my_edge_mesh_ptr];
}
const Mesh& ATData::edge_mesh(long i) const
{
    return *m_edge_mesh_ptrs[i];
}

Simplex ATData::sibling_edge(Mesh* my_edge_mesh_ptr, const Simplex& s)
{
    assert(s.primitive_type() == PrimitiveType::Edge);
    Mesh* sibling_mesh_ptr = sibling_edge_mesh_ptr(my_edge_mesh_ptr);
    std::vector<Simplex> sibling_edge = my_edge_mesh_ptr->map((*sibling_mesh_ptr), s);
    assert(sibling_edge.size() == 1);
    return sibling_edge[0];
}
} // namespace wmtk::components::adaptive_tessellation::operations::internal