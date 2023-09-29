#pragma once

#include <Eigen/Dense>
#include <vector>

namespace wmtk::components::internal {
std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_2d(
    const std::vector<Eigen::Vector2d>& points);
} // namespace wmtk::components::internal