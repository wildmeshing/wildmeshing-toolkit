#pragma once

#include <Eigen/dense>
#include <vector>

namespace wmtk::components::internal {
void delaunay_2d(
    const std::vector<Eigen::Vector2d>& points,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& triangles);
} // namespace wmtk::components::internal