#pragma once

#include <Eigen/Dense>
#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include <wmtk/invariants/Invariant.hpp>
namespace wmtk {
class TriMesh;
}
namespace wmtk::components::rgb::invariants {
class TriMeshRGBSplitInvariant : public Invariant
{
    /**
     * Invariant for rgb refine. Edge color is green (edge_color == 0), and face level of the
     * triangles incident to the edge are the same as the edge level
     */
public:
    TriMeshRGBSplitInvariant(
        const TriMesh& m,
        const attribute::TypedAttributeHandle<int64_t>& face_rgb_state_handle,
        const attribute::TypedAttributeHandle<int64_t>& edge_rgb_state_handle);
    bool before(const simplex::Simplex& t) const override;
    const TriMesh& mesh() const;

private:
    const attribute::TypedAttributeHandle<int64_t> m_face_rgb_state_handle;
    const attribute::TypedAttributeHandle<int64_t> m_edge_rgb_state_handle;
};
} // namespace wmtk::components::rgb::invariants
