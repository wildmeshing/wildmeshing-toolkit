#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::components::internal {
std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_2d(Eigen::Ref<const RowVectors2d> points);
} // namespace wmtk::components::internal
