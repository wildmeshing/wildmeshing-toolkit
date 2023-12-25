#pragma once

#include <Eigen/Core>

namespace wmtk {
template <typename T, int C>
using RowVectors = Eigen::Matrix<T, Eigen::Dynamic, C>;

template <typename T, int R>
using SquareMatrix = Eigen::Matrix<T, R, R>;

template <typename T>
using MatrixX= Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T, int R>
using Vector = Eigen::Matrix<T, R, 1>;
template <typename T>
using VectorX = Vector<T, Eigen::Dynamic>;

template <typename T>
using Vector2 = Vector<T, 2>;
template <typename T>
using Vector3 = Vector<T, 3>;
template <typename T>
using Vector4 = Vector<T, 4>;

template <typename T, int C>
using RowVector = Eigen::Matrix<T, 1, C>;
template <typename T>
using RowVectorX = RowVector<T, Eigen::Dynamic>;

using VectorXl = VectorX<long>;
using Vector2l = Vector<long, 2>;
using Vector3l = Vector<long, 3>;
using Vector4l = Vector<long, 4>;
using Vector5l = Vector<long, 5>;


using RowVector2d = RowVector<double, 2>;
using RowVector3d = RowVector<double, 3>;
using RowVectors2l = RowVectors<long, 2>;
using RowVectors3l = RowVectors<long, 3>;
using RowVectors4l = RowVectors<long, 4>;
using RowVectors6l = RowVectors<long, 6>;
using RowVectors2d = RowVectors<double, 2>;
using RowVectors3d = RowVectors<double, 3>;

} // namespace wmtk
