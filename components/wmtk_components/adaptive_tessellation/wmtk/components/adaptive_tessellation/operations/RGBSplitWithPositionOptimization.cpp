#include "RGBSplitWithPositionOptimization.hpp"

#include <optional>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/AttributeScopeStack.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/pixel_image_triangle_helper.hpp>

namespace wmtk::operations::composite {
RGBSplitWithPositionOptimization::RGBSplitWithPositionOptimization(
    Mesh& m,
    attribute::MeshAttributeHandle& uv_handle,
    std::shared_ptr<wmtk::components::function::utils::Triangle2DTo3DMapping> mapping_ptr,
    attribute::MeshAttributeHandle& triangle_rgb_state_handle,
    attribute::MeshAttributeHandle& edge_rgb_state_handle)
    : Operation(m)
    , m_rgb_split(m, triangle_rgb_state_handle, edge_rgb_state_handle)
    , m_attribute_update(m)
    , m_uv_handle(uv_handle)
    , m_mapping_ptr(mapping_ptr)
{}

std::vector<simplex::Simplex> RGBSplitWithPositionOptimization::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return {simplex};
}

std::vector<simplex::Simplex> RGBSplitWithPositionOptimization::execute(
    const simplex::Simplex& simplex)
{
    auto uv_accessor = mesh().create_accessor(m_uv_handle.as<double>());
    // get the two ends of the edge
    Eigen::Vector2d uv0 = uv_accessor.vector_attribute(simplex.tuple());
    Eigen::Vector2d uv1 =
        uv_accessor.vector_attribute(mesh().switch_tuple(simplex.tuple(), PrimitiveType::Vertex));
    Eigen::Vector2d my_face_opposite_uv = uv_accessor.vector_attribute(
        mesh().switch_tuples(simplex.tuple(), {PrimitiveType::Edge, PrimitiveType::Vertex}));
    std::optional<Eigen::Vector2d> other_face_opposite_uv_opt;
    if (!mesh().is_boundary(simplex)) {
        other_face_opposite_uv_opt = uv_accessor.vector_attribute(mesh().switch_tuples(
            simplex.tuple(),
            {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex}));
    }


    auto split_return = m_rgb_split(simplex);
    if (split_return.empty()) return {};
    assert(split_return.size() == 1);

    std::function update_func = [&](Mesh& mesh, const simplex::Simplex& return_simplex) -> bool {
        Eigen::Vector2d best_uv = wmtk::function::utils::get_best_uv(
            m_mapping_ptr,
            uv0,
            uv1,
            my_face_opposite_uv,
            other_face_opposite_uv_opt);
        // update the new vertex position to be the best position
        uv_accessor.vector_attribute(return_simplex.tuple()) = best_uv;
        return true;
    };
    m_attribute_update.set_function(update_func);
    auto attribute_update_return = m_attribute_update(split_return[0]);


    return attribute_update_return;
}

} // namespace wmtk::operations::composite