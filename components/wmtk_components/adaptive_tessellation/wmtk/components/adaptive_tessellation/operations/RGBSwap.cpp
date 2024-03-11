#include "RGBSwap.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/AttributeScopeStack.hpp>


namespace wmtk::operations::composite {
RGBSwap::RGBSwap(
    Mesh& m,
    attribute::MeshAttributeHandle& triangle_rgb_state_handle,
    attribute::MeshAttributeHandle& edge_rgb_state_handle)
    : Operation(m)
    , m_swap(m)
    , m_triangle_rgb_state_handle(triangle_rgb_state_handle)
    , m_edge_rgb_state_handle(edge_rgb_state_handle)
    , m_triangle_rgb_state_accessor(
          mesh().create_accessor(m_triangle_rgb_state_handle.as<int64_t>()))
    , m_edge_rgb_state_accessor(mesh().create_accessor(m_edge_rgb_state_handle.as<int64_t>()))
{}

std::vector<simplex::Simplex> RGBSwap::unmodified_primitives(const simplex::Simplex& simplex) const
{
    return {simplex};
}

std::vector<simplex::Simplex> RGBSwap::execute(const simplex::Simplex& simplex)
{
    Eigen::Vector2<int64_t> edge_color_level =
        m_edge_rgb_state_accessor.vector_attribute(simplex.tuple());
    Eigen::Vector2<int64_t> my_face_color_level =
        m_triangle_rgb_state_accessor.vector_attribute(simplex.tuple());
    Tuple other_face;
    Eigen::Vector2<int64_t> other_face_color_level(-1, -1);

    other_face = mesh().switch_tuple(simplex.tuple(), PrimitiveType::Triangle);
    other_face_color_level = m_triangle_rgb_state_accessor.vector_attribute(other_face);

    assert(my_face_color_level[0] == 2);
    assert(other_face_color_level[0] == 2);
    assert(my_face_color_level[1] == other_face_color_level[1]);
    auto swap_return = m_swap(simplex);
    if (swap_return.empty()) return {};
    assert(swap_return.size() == 1);
    // now we do the attributes update
    Tuple swap_return_tuple = swap_return[0].tuple();
    m_edge_rgb_state_accessor.vector_attribute(swap_return_tuple) =
        Eigen::Vector2<int64_t>(0, edge_color_level[1] + 1);
    m_triangle_rgb_state_accessor.vector_attribute(swap_return_tuple) =
        Eigen::Vector2<int64_t>(0, my_face_color_level[1] + 1);
    m_triangle_rgb_state_accessor.vector_attribute(
        mesh().switch_tuple(swap_return_tuple, PrimitiveType::Triangle)) =
        Eigen::Vector2<int64_t>(0, other_face_color_level[1] + 1);

    return swap_return;
}

} // namespace wmtk::operations::composite