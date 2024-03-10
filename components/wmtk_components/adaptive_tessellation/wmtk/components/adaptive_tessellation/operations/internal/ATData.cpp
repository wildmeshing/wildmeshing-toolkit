#include "ATData.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
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
// ATData::ATData(
//     std::shared_ptr<Mesh> uv_mesh_ptr,
//     std::shared_ptr<Mesh> position_mesh_ptr,
//     std::vector<std::shared_ptr<Mesh>> edge_mesh_ptrs,
//     std::map<Mesh*, Mesh*> sibling_meshes_map,
//     std::array<std::shared_ptr<image::Image>, 3>& images)
//     : m_uv_mesh_ptr(uv_mesh_ptr)
//     , m_position_mesh_ptr(position_mesh_ptr)
//     , m_edge_mesh_ptrs(edge_mesh_ptrs)
//     , m_sibling_meshes_map(sibling_meshes_map)
//     , m_images(images)
// {
//     m_uv_handle = uv_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
//     // Storing edge lengths
//     m_3d_edge_length_handle =
//         uv_mesh_ptr->get_attribute_handle<double>("edge_length", PrimitiveType::Edge);
//     auto tmp_uv_pt_accessor = uv_mesh_ptr->create_accessor(m_uv_handle.as<double>());
//     auto tmp_edge_length_accessor =
//         uv_mesh_ptr->create_accessor(m_3d_edge_length_handle.as<double>());
//     const auto edges = uv_mesh_ptr->get_all(PrimitiveType::Edge);
//     for (const auto& e : edges) {
//         const auto p0 = tmp_uv_pt_accessor.vector_attribute(e);
//         const auto p1 = tmp_uv_pt_accessor.vector_attribute(uv_mesh_ptr->switch_vertex(e));

//         tmp_edge_length_accessor.scalar_attribute(e) = (p0 - p1).norm();
//     }
// }

// ATData::ATData(
//     std::shared_ptr<Mesh> uv_mesh_ptr,
//     std::shared_ptr<Mesh> position_mesh_ptr,
//     std::array<std::shared_ptr<image::Image>, 3>& images)
//     : m_uv_mesh_ptr(uv_mesh_ptr)
//     , m_position_mesh_ptr(position_mesh_ptr)
//     , m_images(images)
// {
//     auto uv_mesh_map =
//         wmtk::multimesh::same_simplex_dimension_bijection(position_mesh(), uv_mesh());
//     position_mesh().register_child_mesh(m_uv_mesh_ptr, uv_mesh_map);

//     std::set<Tuple> critical_points =
//         ATmultimesh::utils::find_critical_points(uv_mesh(), position_mesh());
//     auto tags = wmtk::multimesh::utils::create_tags(uv_mesh(), critical_points);


//     for (int64_t tag : tags) {
//         m_edge_mesh_ptrs.emplace_back(
//             wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
//                 uv_mesh(),
//                 "edge_tag",
//                 tag,
//                 PrimitiveType::Edge));
//     }
//     std::map<Mesh*, Mesh*> sibling_meshes_map =
//         ATmultimesh::utils::map_sibling_edge_meshes(m_edge_mesh_ptrs);
//     m_sibling_meshes_map = sibling_meshes_map;
//     ATmultimesh::utils::parameterize_all_edge_meshes(
//         static_cast<TriMesh&>(uv_mesh()),
//         m_edge_mesh_ptrs,
//         m_sibling_meshes_map);
//     m_uv_handle = uv_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
//     // Storing edge lengths
//     m_3d_edge_length_handle =
//         uv_mesh_ptr->register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
//     auto tmp_uv_pt_accessor = uv_mesh_ptr->create_accessor(m_uv_handle.as<double>());
//     auto tmp_edge_length_accessor =
//         uv_mesh_ptr->create_accessor(m_3d_edge_length_handle.as<double>());
//     const auto edges = uv_mesh_ptr->get_all(PrimitiveType::Edge);
//     for (const auto& e : edges) {
//         const auto p0 = tmp_uv_pt_accessor.vector_attribute(e);
//         const auto p1 = tmp_uv_pt_accessor.vector_attribute(uv_mesh_ptr->switch_vertex(e));

//         tmp_edge_length_accessor.scalar_attribute(e) = (p0 - p1).norm();
//     }
// }

// ATData::ATData(
//     std::shared_ptr<Mesh> position_mesh_ptr,
//     std::shared_ptr<Mesh> uv_mesh_ptr,
//     const std::filesystem::path& position_path,
//     const std::filesystem::path& normal_path,
//     const std::filesystem::path& height_path)
//     : m_position_mesh_ptr(position_mesh_ptr)
//     , m_uv_mesh_ptr(uv_mesh_ptr)
// {
//     // auto child_map =
//     //     wmtk::multimesh::same_simplex_dimension_bijection(*m_position_mesh_ptr, *m_uv_mesh_ptr);
//     // m_position_mesh_ptr->register_child_mesh(m_uv_mesh_ptr, child_map);
//     initialize_handles();
//     const auto vertices = m_position_mesh_ptr->get_all(PrimitiveType::Vertex);
//     Accessor<double> m_pmesh_xyz_accessor =
//         m_position_mesh_ptr->create_accessor(m_pmesh_xyz_handle.as<double>());

//     Eigen::VectorXd bmin(3);
//     bmin.setConstant(std::numeric_limits<double>::max());
//     Eigen::VectorXd bmax(3);
//     bmax.setConstant(std::numeric_limits<double>::min());
//     for (const auto& v : vertices) {
//         const auto p = m_pmesh_xyz_accessor.vector_attribute(v);
//         for (int64_t d = 0; d < bmax.size(); ++d) {
//             bmin[d] = std::min(bmin[d], p[d]);
//             bmax[d] = std::max(bmax[d], p[d]);
//         }
//     }
//     double max_comp = (bmax - bmin).maxCoeff();
//     Eigen::MatrixXd scene_offset = -bmin;
//     Eigen::MatrixXd scene_extent = bmax - bmin;
//     scene_offset.array() -= (scene_extent.array() - max_comp) * 0.5;
//     Eigen::Vector3d offset = scene_offset;
//     m_images = image::combine_position_normal_texture(
//         max_comp,
//         offset,
//         position_path,
//         normal_path,
//         height_path);
//     std::cout << "+++++ using image sampling !!!!" << std::endl;
// }


ATData::ATData(
    std::shared_ptr<Mesh> position_mesh_ptr,
    std::shared_ptr<Mesh> uv_mesh_ptr,
    std::array<std::shared_ptr<image::Image>, 3>& images)
    : m_position_mesh_ptr(position_mesh_ptr)
    , m_uv_mesh_ptr(uv_mesh_ptr)
    , m_images(images)
{
    // auto child_map =
    //     wmtk::multimesh::same_simplex_dimension_bijection(*m_position_mesh_ptr, *m_uv_mesh_ptr);
    // m_position_mesh_ptr->register_child_mesh(m_uv_mesh_ptr, child_map);

    std::cout << "!!!!! using image sampling !!!!" << std::endl;
    initialize_handles();
}

ATData::ATData(
    std::shared_ptr<Mesh> position_mesh_ptr,
    std::shared_ptr<Mesh> uv_mesh_ptr,
    std::array<std::shared_ptr<image::Sampling>, 3>& funcs)
    : m_position_mesh_ptr(position_mesh_ptr)
    , m_uv_mesh_ptr(uv_mesh_ptr)
    , m_funcs(funcs)
{
    // auto child_map =
    //     wmtk::multimesh::same_simplex_dimension_bijection(*m_position_mesh_ptr, *m_uv_mesh_ptr);
    // m_position_mesh_ptr->register_child_mesh(m_uv_mesh_ptr, child_map);
    std::cout << "----- using analytical functions !!!!" << std::endl;
    initialize_handles();
}

void ATData::initialize_handles()
{
    m_uv_handle = m_uv_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    m_uvmesh_xyz_handle =
        m_uv_mesh_ptr->register_attribute<double>("positions", PrimitiveType::Vertex, 3, true);

    m_distance_error_handle = m_uv_mesh_ptr->register_attribute<double>(
        "distance_error",
        PrimitiveType::Triangle,
        1,
        true);
    m_amips_error_handle =
        m_uv_mesh_ptr->register_attribute<double>("amips_error", PrimitiveType::Triangle, 1, true);
    m_3d_edge_length_handle =
        m_uv_mesh_ptr->register_attribute<double>("3d_edge_length", PrimitiveType::Edge, 1, true);
    m_curved_edge_length_handle = m_uv_mesh_ptr->register_attribute<double>(
        "curved_edge_length",
        PrimitiveType::Edge,
        1,
        true);
    m_face_rgb_state_handle = m_uv_mesh_ptr->register_attribute<int64_t>(
        "face_rgb_state",
        PrimitiveType::Triangle,
        2,
        true);
    m_edge_rgb_state_handle =
        m_uv_mesh_ptr->register_attribute<int64_t>("edge_rgb_state", PrimitiveType::Edge, 2, true);
}

const std::array<std::shared_ptr<image::Image>, 3>& ATData::images() const
{
    return m_images;
}
const std::array<std::shared_ptr<image::Sampling>, 3>& ATData::funcs() const
{
    return m_funcs;
}

MeshAttributeHandle ATData::uv_handle()
{
    return m_uv_handle;
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