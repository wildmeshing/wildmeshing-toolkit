
#pragma once

#include <wmtk/operations/EdgeSplit.hpp>

namespace wmtk::components::rgb::operations {
class RGBEdgeSplit : public wmtk::operations::EdgeSplit
{
public:
    RGBEdgeSplit(
        TriMesh& m,
        const attribute::MeshAttributeHandle& tri_level,
        const attribute::MeshAttributeHandle& edge_level);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
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

private:
<<<<<<< HEAD
    attribute::Accessor<double,TriMesh,1> m_triangle_level_accessor;
    attribute::Accessor<double,TriMesh,1> m_edge_level_accessor;
=======
    attribute::MeshAttributeHandle m_triangle_level_handle;
    attribute::MeshAttributeHandle m_edge_level_handle;
    attribute::Accessor<int64_t, wmtk::TriMesh,2> m_triangle_level_accessor;
    attribute::Accessor<int64_t, wmtk::TriMesh,2> m_edge_level_accessor;
>>>>>>> upstream/mtao/RGB
};
} // namespace wmtk::components::rgb::operations
