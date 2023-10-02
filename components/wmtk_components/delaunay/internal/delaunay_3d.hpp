#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::components::internal {
std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> delaunay_3d(Eigen::Ref<const RowVectors3d> points);
} // namespace wmtk::components::internal
