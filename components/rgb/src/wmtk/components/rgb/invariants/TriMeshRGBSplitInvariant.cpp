#include "TriMeshRGBSplitInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::rgb::invariants {
TriMeshRGBSplitInvariant::TriMeshRGBSplitInvariant(
    const TriMesh& m,
    const TypedAttributeHandle<int64_t>& face_rgb_state_handle,
    const TypedAttributeHandle<int64_t>& edge_rgb_state_handle)
    : Invariant(m, true, false, false)
    , m_face_rgb_state_handle(face_rgb_state_handle)
    , m_edge_rgb_state_handle(edge_rgb_state_handle)
{}
const TriMesh& TriMeshRGBSplitInvariant::mesh() const
{
    return static_cast<const TriMesh&>(Invariant::mesh());
}

namespace {
bool can_refine(const auto& edge_color_level, const auto& my_face_color_level)
{
    if (edge_color_level(0) == 0 && edge_color_level(1) == my_face_color_level(1)) {
        // blue face don't split
        assert(my_face_color_level(0) != 2);
        return true;
    }
    return false;
}
} // namespace
bool TriMeshRGBSplitInvariant::before(const simplex::Simplex& t) const
{
    const attribute::Accessor<int64_t> face_rgb_state_accessor =
        mesh().create_const_accessor<int64_t, 2>(m_face_rgb_state_handle);
    const attribute::Accessor<int64_t> edge_rgb_state_accessor =
        mesh().create_const_accessor<int64_t, 2>(m_edge_rgb_state_handle);

    auto edge_color_level = edge_rgb_state_accessor.const_vector_attribute(t.tuple());
    auto my_face_color_level = face_rgb_state_accessor.const_vector_attribute(t.tuple());
    if (mesh().is_boundary(t)) {
        return can_refine(edge_color_level, my_face_color_level);
    } else {
        auto other_face = mesh().switch_tuple(t.tuple(), PrimitiveType::Triangle);
        Eigen::Vector2<int64_t> other_face_color_level =
            face_rgb_state_accessor.const_vector_attribute<2>(other_face);
        return can_refine(edge_color_level, my_face_color_level) &&
               can_refine(edge_color_level, other_face_color_level);
    }
}
} // namespace wmtk::components::rgb::invariants
