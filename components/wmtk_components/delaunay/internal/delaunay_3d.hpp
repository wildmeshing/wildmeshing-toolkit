#pragma once

#include <Eigen/Dense>
#include <vector>

namespace wmtk::components::internal {
std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_3d(
    const std::vector<Eigen::Vector3d>& points);
} // namespace wmtk::components::internal