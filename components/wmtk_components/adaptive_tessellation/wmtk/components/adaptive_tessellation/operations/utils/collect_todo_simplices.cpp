#include "collect_todo_simplices.hpp"
namespace wmtk::components::operations::utils {
std::vector<wmtk::simplex::Simplex> get_all_edges_of_all_triangles_with_triangle_filter(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<double>& face_attr_accessor,
    double face_attr_filter_threshold)
{
    const auto tups = uv_mesh_ptr->get_all(PrimitiveType::Triangle);
    std::vector<wmtk::simplex::Simplex> edge_simplices;
    for (const auto& tup : tups) {
        double face_attr = face_attr_accessor.scalar_attribute(tup);
        if (face_attr < face_attr_filter_threshold) {
            continue;
        }
        // get all the edges simplices of each triangle tuple
        // current edge
        edge_simplices.emplace_back(wmtk::simplex::Simplex(PrimitiveType::Edge, tup));
        auto next_edge = uv_mesh_ptr->switch_tuple(tup, PrimitiveType::Edge);
        edge_simplices.emplace_back(wmtk::simplex::Simplex(PrimitiveType::Edge, next_edge));
        auto next_next_edge =
            uv_mesh_ptr->switch_tuples(tup, {PrimitiveType::Vertex, PrimitiveType::Edge});
        edge_simplices.emplace_back(wmtk::simplex::Simplex(PrimitiveType::Edge, next_next_edge));
    }
    return edge_simplices;
}
} // namespace wmtk::components::operations::utils