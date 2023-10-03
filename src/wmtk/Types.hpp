#pragma once

#include <Eigen/Core>

namespace wmtk {
using RowVectors2l = Eigen::Matrix<long, Eigen::Dynamic, 2>;
using RowVectors3l = Eigen::Matrix<long, Eigen::Dynamic, 3>;
using VectorXl = Eigen::Matrix<long, Eigen::Dynamic, 1>;
using RowVectors4l = Eigen::Matrix<long, Eigen::Dynamic, 4>;
using RowVectors6l = Eigen::Matrix<long, Eigen::Dynamic, 6>;
using RowVectors3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;

template <typename T, int R>
using Vector = Eigen::Matrix<T, R, 1>;
template <typename T>
using VectorX = Vector<T, Eigen::Dynamic>;
} // namespace wmtk
