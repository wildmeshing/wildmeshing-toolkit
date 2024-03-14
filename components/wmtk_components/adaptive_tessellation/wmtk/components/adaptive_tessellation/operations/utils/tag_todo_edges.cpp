#include "tag_todo_edges.hpp"

namespace wmtk::components::operations::utils {
void tag_todo_edges_of_each_face(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::MeshAttributeHandle& edge_todo_handle,
    wmtk::attribute::Accessor<double>& face_attr_accessor,
    wmtk::attribute::Accessor<double>& edge_attr_accessor,
    double face_attr_filter_threshold)
{
    // create the accessor for the the edge_todo_handle
    auto edge_todo_accessor = uv_mesh_ptr->create_accessor(edge_todo_handle.as<int64_t>());
    const auto faces = uv_mesh_ptr->get_all(wmtk::PrimitiveType::Triangle);
    for (const auto& face : faces) {
        double face_attr = face_attr_accessor.scalar_attribute(face);
        if (face_attr < face_attr_filter_threshold) {
            continue;
        }
        // tag the longest edge as todo
        auto edge = face;
        auto next_edge = uv_mesh_ptr->switch_tuple(edge, PrimitiveType::Edge);
        auto next_next_edge =
            uv_mesh_ptr->switch_tuples(edge, {PrimitiveType::Vertex, PrimitiveType::Edge});

        // find the one that has largest edge_attr

        Tuple max_edge_tuple;
        double max_edge_attr = -1;
        for (const auto& e : {edge, next_edge, next_next_edge}) {
            double edge_attr = edge_attr_accessor.scalar_attribute(e);
            if (edge_attr > max_edge_attr) {
                max_edge_attr = edge_attr;
                max_edge_tuple = e;
            }
        }

        // set the todo tag of the max_edge_tuple to 1
        edge_todo_accessor.scalar_attribute(max_edge_tuple) = 1;
    }
}
} // namespace wmtk::components::operations::utils
