#include "tag_todo_edges.hpp"
#include <Eigen/Dense>
namespace wmtk::components::operations::utils {
void tag_longest_edge_of_all_faces(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    wmtk::attribute::Accessor<double>& face_attr_accessor,
    wmtk::attribute::Accessor<double>& edge_attr_accessor,
    double face_attr_filter_threshold)
{
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

void tag_secondary_split_edges(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<int64_t>& face_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    Tuple& edge)

{
    // get the {color, level} of the edge
    auto edge_color_level = edge_rgb_state_accessor.vector_attribute(edge);
    // case switch
    switch (edge_color_level[0]) {
    case 0:
        // case 0: the edge is a (green, l)
        // a. it's adjacent to (red, l-1). Then tag their (green, l-1) edges
        // b. it's adjacent to (blue, l-1). Find the (red, l-1) edge of the blue face.
        // This edge must be  adjacent to a (red, l-1) face. Find that red face and tag its (green,
        // l-1) edge as todo
        tag_green_edge_secondary_edges(
            uv_mesh_ptr,
            face_rgb_state_accessor,
            edge_rgb_state_accessor,
            edge_todo_accessor,
            edge);
        if (!uv_mesh_ptr->is_boundary(wmtk::simplex::Simplex(PrimitiveType::Edge, edge))) {
            // if the edge is a boundary edge, tag the other edge as todo
            Tuple other_edge = uv_mesh_ptr->switch_tuple(edge, PrimitiveType::Triangle);
            tag_green_edge_secondary_edges(
                uv_mesh_ptr,
                face_rgb_state_accessor,
                edge_rgb_state_accessor,
                edge_todo_accessor,
                other_edge);
        }
        break;
    case 1: {
        // case 1: the edge is a (red, l)
        // it can be adjcent to (red,l) or (blue,l) faces. Only the red face can have (green,l) edge
        // Tag the red face's (green, l) edges as todo

        tag_red_edge_secondary_edges(
            uv_mesh_ptr,
            face_rgb_state_accessor,
            edge_rgb_state_accessor,
            edge_todo_accessor,
            edge);
        if (!uv_mesh_ptr->is_boundary(wmtk::simplex::Simplex(PrimitiveType::Edge, edge))) {
            // if the edge is a boundary edge, tag the other edge as todo
            Tuple other_edge = uv_mesh_ptr->switch_tuple(edge, PrimitiveType::Triangle);
            tag_red_edge_secondary_edges(
                uv_mesh_ptr,
                face_rgb_state_accessor,
                edge_rgb_state_accessor,
                edge_todo_accessor,
                other_edge);
        }
        break;
    }
    default: throw std::runtime_error("secondary edge tag invalid edge color");
    }
}
void tag_green_edge_secondary_edges(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<int64_t>& face_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    Tuple& edge)
{
    Eigen::Vector2<int64_t> face_color_level = face_rgb_state_accessor.vector_attribute(edge);
    switch (face_color_level[0]) {
    case 0: {
        // green face has nothing to be tagged
        break;
    }
    case 1: {
        // (red, l-1) face tag its (green, l-1) edges as todo
        assert(face_color_level[1] == edge_rgb_state_accessor.vector_attribute(edge)[1] - 1);
        tag_red_l_face_green_l_edge(
            uv_mesh_ptr,
            face_rgb_state_accessor,
            edge_rgb_state_accessor,
            edge_todo_accessor,
            edge);
        break;
    }
    case 2: {
        // find red edge of the blue face
        Tuple ear0 = uv_mesh_ptr->switch_tuple(edge, PrimitiveType::Edge);
        Tuple ear1 = uv_mesh_ptr->switch_tuples(edge, {PrimitiveType::Vertex, PrimitiveType::Edge});
        Tuple red_edge = edge_rgb_state_accessor.vector_attribute(ear0)[0] == 1 ? ear0 : ear1;
        assert(edge_rgb_state_accessor.vector_attribute(red_edge)[0] == 1);
        if (uv_mesh_ptr->is_boundary(wmtk::simplex::Simplex(PrimitiveType::Edge, red_edge))) {
            // this can't happen
            assert(false);
            break;
        }
        // if the red edge is not a boundary edge find the other face
        Tuple other_face = uv_mesh_ptr->switch_tuple(red_edge, PrimitiveType::Triangle);
        if (face_rgb_state_accessor.vector_attribute(other_face)[0] == 2) {
            // it's blueface/ red edge / blueface config
            // nothing can be tagged
            break;
        }
        assert(face_rgb_state_accessor.vector_attribute(other_face)[0] == 1);
        // tag the red face's (green, l-1) edges as todo
        tag_red_l_face_green_l_edge(
            uv_mesh_ptr,
            face_rgb_state_accessor,
            edge_rgb_state_accessor,
            edge_todo_accessor,
            other_face);
        break;
    }


    default: throw std::runtime_error("tag_green_edge_secondary_edges: invalid face color");
    }
}
void tag_red_edge_secondary_edges(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<int64_t>& face_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    Tuple& edge)
{
    Eigen::Vector2<int64_t> face_color_level = face_rgb_state_accessor.vector_attribute(edge);
    switch (face_color_level[0]) {
    case 0: {
        // red edge can't be adjacent to green face
        assert(false);
        break;
    }
    case 1: {
        // the face is a (red, l)
        // tag its (green, l) edges as todo
        tag_red_l_face_green_l_edge(
            uv_mesh_ptr,
            face_rgb_state_accessor,
            edge_rgb_state_accessor,
            edge_todo_accessor,
            edge);
        break;
    }
    case 2: {
        // blue face has no green edge
        break;
    }
    default: throw std::runtime_error("tag_red_edge_secondary_edges: invalid face color");
    }
}

void tag_red_l_face_green_l_edge(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<int64_t>& face_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    Tuple& edge)
{
    Eigen::Vector2<int64_t> face_color_level = face_rgb_state_accessor.vector_attribute(edge);
    assert(face_color_level[0] == 1);
    Tuple red_edge = edge;
    auto edge_color_level = edge_rgb_state_accessor.vector_attribute(red_edge);
    Tuple ear0 = uv_mesh_ptr->switch_tuple(edge, PrimitiveType::Edge);
    Tuple ear1 = uv_mesh_ptr->switch_tuples(edge, {PrimitiveType::Vertex, PrimitiveType::Edge});
    auto ear0_color_level = edge_rgb_state_accessor.vector_attribute(ear0);
    auto ear1_color_level = edge_rgb_state_accessor.vector_attribute(ear1);
    if (ear0_color_level[0] == 1) {
        assert(edge_color_level[0] == 0);
        red_edge = ear0;
        ear0 = edge;
        ear0_color_level = edge_rgb_state_accessor.vector_attribute(ear0);
    } else if (ear1_color_level[0] == 1) {
        assert(edge_color_level[0] == 0);
        red_edge = ear1;
        ear1 = edge;
        ear1_color_level = edge_rgb_state_accessor.vector_attribute(ear1);
    }
    // they must both be green

    assert(ear0_color_level[0] == 0);
    assert(ear1_color_level[0] == 0);
    if (ear0_color_level[1] == face_color_level[1]) {
        edge_todo_accessor.scalar_attribute(ear0) = 1;
    } else if (ear1_color_level[1] == face_color_level[1]) {
        edge_todo_accessor.scalar_attribute(ear1) = 1;
    } else {
        // one the ear edge of the red face must have the same level as the face
        throw std::runtime_error("tag_red_face_secondary_edges: invalid edge level");
    }
    // once unit tested can be replaced by the following
    // edge_rgb_state_accessor.vector_attribute(ear0)[1] == face_color_level[1]
    //     ? edge_todo_accessor.scalar_attribute(ear0) = 1
    //     : edge_todo_accessor.scalar_attribute(ear1) = 1;
}
} // namespace wmtk::components::operations::utils
