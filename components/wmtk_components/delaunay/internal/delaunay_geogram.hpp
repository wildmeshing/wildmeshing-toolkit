#pragma once

#include <Eigen/Dense>
#include <vector>

namespace wmtk::components::internal {
template <typename VectorT>
std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_geogram(const std::vector<VectorT>& points);
} // namespace wmtk::components::internal