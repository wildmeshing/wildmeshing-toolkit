#include "RGBEdgeSwap.hpp"
namespace wmtk::components::rgb::operations {
RGBTriEdgeSwap::RGBTriEdgeSwap(
    TriMesh& m,
    const attribute::MeshAttributeHandle& triangle_rgb_state_handle,
    const attribute::MeshAttributeHandle& edge_rgb_state_handle)
    : TriEdgeSwap(m)
    , m_triangle_level_handle(triangle_rgb_state_handle)
    , m_edge_level_handle(edge_rgb_state_handle)
{}

TriMesh& RGBTriEdgeSwap::mesh()
{
    return static_cast<TriMesh&>(TriEdgeSwap::mesh());
}
const TriMesh& RGBTriEdgeSwap::mesh() const
{
    return static_cast<const TriMesh&>(TriEdgeSwap::mesh());
}

std::vector<simplex::Simplex> RGBTriEdgeSwap::execute(const simplex::Simplex& simplex)
{
    assert(!mesh().is_boundary(simplex));
    auto tri_acc = mesh().create_accessor<int64_t, 2>(m_triangle_level_handle);
    auto edge_acc = mesh().create_accessor<int64_t, 2>(m_edge_level_handle);

    auto edge_color_level = edge_acc.const_vector_attribute(simplex.tuple());
    auto my_face_color_level = tri_acc.const_vector_attribute<2>(simplex.tuple());

    Tuple other_face = mesh().switch_tuple(simplex.tuple(), PrimitiveType::Triangle);
    auto other_face_color_level = tri_acc.vector_attribute(other_face);

    assert(my_face_color_level[0] == 2);
    assert(other_face_color_level[0] == 2);
    assert(my_face_color_level[1] == other_face_color_level[1]);
    assert(edge_color_level[0] == 1);
    auto swap_return = TriEdgeSwap::execute(simplex);
    if (swap_return.empty()) return {};
    assert(swap_return.size() == 1);
    // now we do the attributes update
    Tuple swap_return_tuple = swap_return[0].tuple();
    edge_acc.vector_attribute(swap_return_tuple) =
        Eigen::Vector2<int64_t>(0, edge_color_level(1) + 1);
    tri_acc.vector_attribute(swap_return_tuple) =
        Eigen::Vector2<int64_t>(0, my_face_color_level(1) + 1);
    tri_acc.vector_attribute(mesh().switch_tuple(swap_return_tuple, PrimitiveType::Triangle)) =
        Eigen::Vector2<int64_t>(0, other_face_color_level(1) + 1);
    return swap_return;
    //
}
} // namespace wmtk::components::rgb::operations
