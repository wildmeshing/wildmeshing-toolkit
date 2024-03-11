#pragma once

#include <Eigen/Dense>
#include <wmtk/attribute/AttributeHandle.hpp>
#include <wmtk/invariants/Invariant.hpp>
namespace wmtk {
class RGBSwapInvariant : public Invariant
{
    /**
     * Invariant for rgb refine. Edge color is green (edge_color == 0), and face level of the
     * triangles incident to the edge are the same as the edge level
     */
public:
    RGBSwapInvariant(
        const Mesh& m,
        const TypedAttributeHandle<int64_t>& face_rgb_state_handle,
        const TypedAttributeHandle<int64_t>& edge_rgb_state_handle);
    bool before(const simplex::Simplex& t) const override;

private:
    bool can_swap(
        const Eigen::Vector2<int64_t>& edge_color_level,
        const Eigen::Vector2<int64_t>& face_color_level) const;
    const TypedAttributeHandle<int64_t> m_face_rgb_state_handle;
    const TypedAttributeHandle<int64_t> m_edge_rgb_state_handle;
};
} // namespace wmtk