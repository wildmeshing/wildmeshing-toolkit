#pragma once

#include <vector>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/Primitive.hpp>

namespace wmtk::components::internal {
wmtk::TriMesh extract_subset_2d(
    const std::vector<Eigen::Vector2d>& points,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& triangles, std::vector<size_t> tag);
} // namespace wmtk::components::internal