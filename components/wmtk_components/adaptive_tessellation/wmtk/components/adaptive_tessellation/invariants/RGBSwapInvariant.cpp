#include "RGBSwapInvariant.hpp"

namespace wmtk {
RGBSwapInvariant::RGBSwapInvariant(
    const Mesh& m,
    const TypedAttributeHandle<int64_t>& face_rgb_state_handle,
    const TypedAttributeHandle<int64_t>& edge_rgb_state_handle)
    : m_face_rgb_state_handle(face_rgb_state_handle)
    , m_edge_rgb_state_handle(edge_rgb_state_handle)
{}

bool RGBSwapInvariant::can_swap(
    const Eigen::Vector2<int64_t>& edge_color_level,
    const Eigen::Vector2<int64_t>& face_color_level) const
{
    // must be blue face
    if (face_color_level(0) != 2) return false;

    if (edge_color_level(1) == face_color_level(1)) {
        return true;
    }
    return false;
}
bool RGBSwapInvariant::before(const simplex::Simplex& t) const
{
    const attribute::Accessor<int64_t> face_rbg_state_accessor =
        mesh().create_const_accessor<int64_t>(m_face_rgb_state_handle);
    const attribute::Accessor<int64_t> edge_rbg_state_accessor =
        mesh().create_const_accessor<int64_t>(m_edge_rgb_state_handle);

    Eigen::Vector2<int64_t> edge_color_level =
        edge_rbg_state_accessor.const_vector_attribute<2>(t.tuple());
    Eigen::Vector2<int64_t> my_face_color_level =
        face_rbg_state_accessor.const_vector_attribute<2>(t.tuple());
    if (mesh().is_boundary(t)) {
        return false;
    }

    auto other_face = mesh().switch_tuple(t.tuple(), PrimitiveType::Triangle);
    Eigen::Vector2<int64_t> other_face_color_level =
        face_rbg_state_accessor.const_vector_attribute<2>(other_face);

    // red edge
    if (edge_color_level[0] != 1) return false;

    Eigen::Vector2<int64_t> other_face_color_level =
        face_rbg_state_accessor.const_vector_attribute<2>(other_face);
    return can_swap(edge_color_level, my_face_color_level) &&
           can_swap(edge_color_level, other_face_color_level);
}
} // namespace wmtk