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
void tag_todo_edges_of_each_face(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::MeshAttributeHandle& edge_todo_handle,
    wmtk::attribute::Accessor<double>& face_attr_accessor,
    wmtk::attribute::Accessor<double>& edge_attr_accessor,
    double face_attr_filter_threshold);
} // namespace wmtk::components::operations::utils