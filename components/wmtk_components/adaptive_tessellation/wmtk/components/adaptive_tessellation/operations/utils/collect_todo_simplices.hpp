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
std::vector<wmtk::simplex::Simplex> get_all_edges_of_all_triangles_with_triangle_filter(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<double>& face_attr_accessor,
    double face_attr_filter_threshold);
} // namespace wmtk::components::operations::utils