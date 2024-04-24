#pragma once

#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>

namespace wmtk::operations::composite {

class RGBSplit : public Operation
{
public:
    RGBSplit(
        Mesh& m,
        attribute::MeshAttributeHandle& uv_handle,
        attribute::MeshAttributeHandle& best_point_handle,
        attribute::MeshAttributeHandle& triangle_rgb_state_handle,
        attribute::MeshAttributeHandle& edge_rgb_state_handle);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    inline EdgeSplit& split() { return m_split; }

protected:
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    EdgeSplit m_split;

    attribute::MeshAttributeHandle m_uv_handle;
    attribute::MeshAttributeHandle m_best_point_handle;
    attribute::MeshAttributeHandle m_triangle_rgb_state_handle;
    attribute::MeshAttributeHandle m_edge_rgb_state_handle;
    attribute::Accessor<int64_t> m_triangle_rgb_state_accessor;
    attribute::Accessor<int64_t> m_edge_rgb_state_accessor;

    //     split edge update:
    // old edge == 0 green , l → new_edge(0 green, l+1)
    void split_edge_update(
        const Tuple& split_return_edge,
        const Eigen::Vector2<int64_t>& old_edge_color_level);

    //     split rib edge update:
    // if old_face color == 0 green → rib_edge (1 red, l)
    //                      1  red → rib_edge (0 green, l+1)
    void rib_edge_new_attribute_based_on_old_face(
        const Tuple& rib_edge,
        const Eigen::Vector2<int64_t>& old_face_color_level);

    // split face update:
    // if old_face ==  0 green , l →2x new faces  (1 red, l)
    //                 1 red ,   l → if ear edge is  0 green, new face (0 green, l+1)
    //                               if ear edge is  1 red, new face (2 blue, l)
    void face_update(
        const Tuple& split_return,
        const Eigen::Vector2<int64_t>& old_face_color_level);

    void red_face_update_based_on_ear_edge(
        const Tuple& new_face,
        const Tuple& ear_edge,
        const Eigen::Vector2<int64_t>& old_face_color_level);
};

} // namespace wmtk::operations::composite
