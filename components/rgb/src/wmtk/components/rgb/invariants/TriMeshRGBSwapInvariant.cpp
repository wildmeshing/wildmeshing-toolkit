#include "TriMeshRGBSwapInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::rgb::invariants {
TriMeshRGBSwapInvariant::TriMeshRGBSwapInvariant(
    const TriMesh& m,
    const attribute::TypedAttributeHandle<int64_t>& face_rgb_state_handle,
    const attribute::TypedAttributeHandle<int64_t>& edge_rgb_state_handle)
    : Invariant(m, true, false, false)
    , m_face_rgb_state_handle(face_rgb_state_handle)
    , m_edge_rgb_state_handle(edge_rgb_state_handle)
{}

namespace {
bool can_swap(const auto& edge_color_level, const auto& face_color_level)
{
    // must be blue face
    if (face_color_level(0) != 2) {
        return false;
    } else if (edge_color_level(1) == face_color_level(1)) {
        return true;
    }
    return false;
}
} // namespace
bool TriMeshRGBSwapInvariant::before(const simplex::Simplex& t) const
{
    const attribute::Accessor<int64_t> face_rgb_state_accessor =
        mesh().create_const_accessor<int64_t, 2>(m_face_rgb_state_handle);
    const attribute::Accessor<int64_t> edge_rgb_state_accessor =
        mesh().create_const_accessor<int64_t, 2>(m_edge_rgb_state_handle);

    auto edge_color_level = edge_rgb_state_accessor.const_vector_attribute(t.tuple());
    auto my_face_color_level = face_rgb_state_accessor.const_vector_attribute(t.tuple());
    if (mesh().is_boundary(t)) {
        return false;
    } else {
        auto other_face = mesh().switch_tuple(t.tuple(), PrimitiveType::Triangle);
        Eigen::Vector2<int64_t> other_face_color_level =
            face_rgb_state_accessor.const_vector_attribute<2>(other_face);

        // red edge
        if (edge_color_level[0] != 1) return false;

        return can_swap(edge_color_level, my_face_color_level) &&
               can_swap(edge_color_level, other_face_color_level);
    }
}
} // namespace wmtk::components::rgb::invariants
