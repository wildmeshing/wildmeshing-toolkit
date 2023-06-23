#pragma once

#include <Eigen/Core>

namespace wmtk {
using RowVectors3l = Eigen::Matrix<long, Eigen::Dynamic, 3>;
using VectorXl = Eigen::Matrix<long, Eigen::Dynamic, 1>;
using RowVectors4l = Eigen::Matrix<long, Eigen::Dynamic, 4>;
using RowVectors3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;
} // namespace wmtk