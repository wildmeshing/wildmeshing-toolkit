#pragma once
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/simplex/Simplex.hpp>
using namespace wmtk;
using namespace wmtk::attribute;
namespace wmtk::components::operations::utils {
// tag longest edge of the worst triangle for example
void tag_longest_edge_of_all_faces(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    wmtk::attribute::Accessor<double>& face_attr_accessor,
    wmtk::attribute::Accessor<double>& edge_attr_accessor,
    double face_attr_filter_threshold);

void tag_secondary_split_edges(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<int64_t>& face_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    Tuple& edge);
void tag_green_edge_secondary_edges(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<int64_t>& face_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    Tuple& edge);
void tag_red_edge_secondary_edges(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<int64_t>& face_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    Tuple& edge);
void tag_red_l_face_green_l_edge(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<int64_t>& face_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_rgb_state_accessor,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    Tuple& edge);


} // namespace wmtk::components::operations::utils