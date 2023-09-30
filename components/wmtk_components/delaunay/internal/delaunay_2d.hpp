#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::components::internal {
void delaunay_2d(
    Eigen::Ref<const RowVectors2d> points,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& triangles);
} // namespace wmtk::components::internal
