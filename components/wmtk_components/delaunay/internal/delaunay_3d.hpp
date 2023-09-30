#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::components::internal {
void delaunay_3d(
    Eigen::Ref<const RowVectors3d> points,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& tetrahedra);
} // namespace wmtk::components::internal
