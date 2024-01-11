#include "ATData.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/attribute/MutableAccessor.hpp>
#include <wmtk/components/adaptive_tessellation/multimesh/utils/edge_meshes_parameterization.hpp>
#include <wmtk/components/adaptive_tessellation/multimesh/utils/find_critical_points.hpp>
#include <wmtk/components/adaptive_tessellation/multimesh/utils/map_sibling_edge_meshes.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/utils/create_tag.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>

namespace ATmultimesh = wmtk::components::multimesh;
namespace wmtk::components::operations::internal {
using namespace wmtk;
using namespace wmtk::attribute;
ATData::ATData(
    std::shared_ptr<Mesh> uv_mesh_ptr,
    std::shared_ptr<Mesh> position_mesh_ptr,
    std::vector<std::shared_ptr<Mesh>> edge_mesh_ptrs,
    std::map<Mesh*, Mesh*> sibling_meshes_map,
    std::array<std::shared_ptr<image::Image>, 3>& images)
    : m_uv_mesh_ptr(uv_mesh_ptr)
    , m_position_mesh_ptr(position_mesh_ptr)
    , m_edge_mesh_ptrs(edge_mesh_ptrs)
    , m_sibling_meshes_map(sibling_meshes_map)
    , m_images(images)
{
    m_uv_handle = uv_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    // Storing edge lengths
    m_3d_edge_length_handle =
        uv_mesh_ptr->get_attribute_handle<double>("edge_length", PrimitiveType::Edge);
    auto tmp_uv_pt_accessor = uv_mesh_ptr->create_accessor(m_uv_handle.as<double>());
    auto tmp_edge_length_accessor =
        uv_mesh_ptr->create_accessor(m_3d_edge_length_handle.as<double>());
    const auto edges = uv_mesh_ptr->get_all(PrimitiveType::Edge);
    for (const auto& e : edges) {
        const auto p0 = tmp_uv_pt_accessor.vector_attribute(e);
        const auto p1 = tmp_uv_pt_accessor.vector_attribute(uv_mesh_ptr->switch_vertex(e));

        tmp_edge_length_accessor.scalar_attribute(e) = (p0 - p1).norm();
    }
}

ATData::ATData(
    std::shared_ptr<Mesh> uv_mesh_ptr,
    std::shared_ptr<Mesh> position_mesh_ptr,
    std::array<std::shared_ptr<image::Image>, 3>& images)
    : m_uv_mesh_ptr(uv_mesh_ptr)
    , m_position_mesh_ptr(position_mesh_ptr)
    , m_images(images)
{
    auto uv_mesh_map =
        wmtk::multimesh::same_simplex_dimension_bijection(position_mesh(), uv_mesh());
    position_mesh().register_child_mesh(m_uv_mesh_ptr, uv_mesh_map);

    std::set<Tuple> critical_points =
        ATmultimesh::utils::find_critical_points(uv_mesh(), position_mesh());
    auto tags = wmtk::multimesh::utils::create_tags(uv_mesh(), critical_points);


    for (int64_t tag : tags) {
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
        static_cast<TriMesh&>(uv_mesh()),
        m_edge_mesh_ptrs,
        m_sibling_meshes_map);
    m_uv_handle = uv_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    // Storing edge lengths
    m_3d_edge_length_handle =
        uv_mesh_ptr->register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto tmp_uv_pt_accessor = uv_mesh_ptr->create_accessor(m_uv_handle.as<double>());
    auto tmp_edge_length_accessor =
        uv_mesh_ptr->create_accessor(m_3d_edge_length_handle.as<double>());
    const auto edges = uv_mesh_ptr->get_all(PrimitiveType::Edge);
    for (const auto& e : edges) {
        const auto p0 = tmp_uv_pt_accessor.vector_attribute(e);
        const auto p1 = tmp_uv_pt_accessor.vector_attribute(uv_mesh_ptr->switch_vertex(e));

        tmp_edge_length_accessor.scalar_attribute(e) = (p0 - p1).norm();
    }
}

ATData::ATData(
    std::shared_ptr<Mesh> uv_mesh_ptr,
    std::array<std::shared_ptr<image::Image>, 3>& images)
    : m_uv_mesh_ptr(uv_mesh_ptr)
    , m_images(images)
{
    m_uv_handle = uv_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    // Storing edge lengths
    m_3d_edge_length_handle =
        uv_mesh_ptr->register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    m_position_handle =
        uv_mesh_ptr->register_attribute<double>("position", PrimitiveType::Vertex, 3);

    auto tmp_3d_pt_accessor = m_uv_mesh_ptr->create_accessor(m_position_handle.as<double>());
    auto tmp_edge_length_accessor =
        uv_mesh_ptr->create_accessor(m_3d_edge_length_handle.as<double>());
    const auto edges = uv_mesh_ptr->get_all(PrimitiveType::Edge);
    for (const auto& e : edges) {
        const auto p0 = tmp_3d_pt_accessor.vector_attribute(e);
        const auto p1 = tmp_3d_pt_accessor.vector_attribute(uv_mesh_ptr->switch_vertex(e));

        tmp_edge_length_accessor.scalar_attribute(e) = (p0 - p1).norm();
    }
}

ATData::ATData(
    std::shared_ptr<Mesh> uv_mesh_ptr,
    std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3>& funcs)
    : m_uv_mesh_ptr(uv_mesh_ptr)
    , m_funcs(funcs)
{
    m_uv_handle = m_uv_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    // Storing edge lengths
    m_3d_edge_length_handle =
        m_uv_mesh_ptr->register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    m_position_handle =
        uv_mesh_ptr->register_attribute<double>("position", PrimitiveType::Vertex, 3);

    auto tmp_3d_pt_accessor = m_uv_mesh_ptr->create_accessor(m_position_handle.as<double>());
    auto tmp_edge_length_accessor =
        m_uv_mesh_ptr->create_accessor(m_3d_edge_length_handle.as<double>());
    const auto edges = m_uv_mesh_ptr->get_all(PrimitiveType::Edge);
    for (const auto& e : edges) {
        const auto p0 = tmp_3d_pt_accessor.vector_attribute(e);
        const auto p1 = tmp_3d_pt_accessor.vector_attribute(m_uv_mesh_ptr->switch_vertex(e));

        tmp_edge_length_accessor.scalar_attribute(e) = (p0 - p1).norm();
    }
}

const std::array<std::shared_ptr<image::Image>, 3>& ATData::images() const
{
    return m_images;
}
const std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3>& ATData::funcs() const
{
    return m_funcs;
}

MeshAttributeHandle ATData::uv_handle()
{
    return m_uv_handle;
}
MeshAttributeHandle ATData::edge_len_handle()
{
    return m_3d_edge_length_handle;
}
Mesh& ATData::uv_mesh() const
{
    return *m_uv_mesh_ptr;
}

Mesh& ATData::position_mesh() const
{
    return *m_position_mesh_ptr;
}

std::shared_ptr<Mesh> ATData::uv_mesh_ptr() const
{
    return m_uv_mesh_ptr;
}
std::shared_ptr<Mesh> ATData::position_mesh_ptr() const
{
    return m_position_mesh_ptr;
}
Mesh* ATData::sibling_edge_mesh_ptr(Mesh* my_edge_mesh_ptr)
{
    return m_sibling_meshes_map[my_edge_mesh_ptr];
}
std::shared_ptr<Mesh> ATData::edge_mesh_i_ptr(int64_t i) const
{
    return m_edge_mesh_ptrs[i];
}
int64_t ATData::num_edge_meshes() const
{
    return m_edge_mesh_ptrs.size();
}
Simplex ATData::sibling_edge(Mesh* my_edge_mesh_ptr, const Simplex& s)
{
    assert(s.primitive_type() == PrimitiveType::Edge);
    Mesh* sibling_mesh_ptr = sibling_edge_mesh_ptr(my_edge_mesh_ptr);
    std::vector<Simplex> sibling_edge = my_edge_mesh_ptr->map((*sibling_mesh_ptr), s);
    assert(sibling_edge.size() == 1);
    return sibling_edge[0];
}
} // namespace wmtk::components::operations::internal