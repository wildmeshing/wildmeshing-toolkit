#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components {

std::shared_ptr<Mesh> delaunay(
    const PointMesh& point_cloud,
    const attribute::MeshAttributeHandle& pts_attr,
    const std::string& output_pos_attr_name = "vertices");

} // namespace wmtk::components
