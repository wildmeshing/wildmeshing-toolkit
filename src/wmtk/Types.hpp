#pragma once

#include <Eigen/Core>

namespace wmtk {
template <typename T, int C>
using RowVectors = Eigen::Matrix<T, Eigen::Dynamic, C>;
using RowVectors3l = Eigen::Matrix<long, Eigen::Dynamic, 3>;
using RowVectors2l = Eigen::Matrix<long, Eigen::Dynamic, 2>;
using VectorXl = Eigen::Matrix<long, Eigen::Dynamic, 1>;
using RowVectors4l = Eigen::Matrix<long, Eigen::Dynamic, 4>;
using RowVectors6l = Eigen::Matrix<long, Eigen::Dynamic, 6>;
using RowVectors3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;

template <typename T, int R>
using Vector = Eigen::Matrix<T, R, 1>;
template <typename T>
using VectorX = Vector<T, Eigen::Dynamic>;

template <typename T, int C>
using RowVector = Eigen::Matrix<T, 1, C>;
template <typename T>
using RowVectorX = RowVector<T, Eigen::Dynamic>;

using VectorXl = VectorX<long>;
using RowVector2d = RowVector<double, 2>;
using RowVector3d = RowVector<double, 3>;
using RowVectors2l = RowVectors<long, 2>;
using RowVectors3l = RowVectors<long, 3>;
using RowVectors4l = RowVectors<long, 4>;
using RowVectors6l = RowVectors<long, 6>;
using RowVectors2d = RowVectors<double, 2>;
using RowVectors3d = RowVectors<double, 3>;

} // namespace wmtk
