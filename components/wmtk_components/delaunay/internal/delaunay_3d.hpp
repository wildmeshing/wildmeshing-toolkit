#pragma once

#include <Eigen/dense>
#include <vector>

namespace wmtk::components::internal {
void delaunay_3d(
    const std::vector<Eigen::Vector3d>& points,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& tetrahedra);
} // namespace wmtk::components::internal