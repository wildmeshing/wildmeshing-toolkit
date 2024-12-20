#pragma once

#include <Eigen/Core>
#include <wmtk/utils/Rational.hpp>

namespace wmtk {
template <typename T, int C>
using RowVectors = Eigen::Matrix<T, Eigen::Dynamic, C>;

template <typename T, int R>
using SquareMatrix = Eigen::Matrix<T, R, R>;

template <typename T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T, int R>
using Vector = Eigen::Matrix<T, R, 1>;
template <typename T>
using VectorX = Vector<T, Eigen::Dynamic>;

template <typename T>
using Vector1 = Vector<T, 1>;
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

using VectorXl = VectorX<int64_t>;
using Vector2l = Vector<int64_t, 2>;
using Vector3l = Vector<int64_t, 3>;
using Vector4l = Vector<int64_t, 4>;
using Vector5l = Vector<int64_t, 5>;

using Vector3d = Vector<double, 3>;

using Vector3r = Vector<Rational, 3>;
using Vector2r = Vector<Rational, 2>;

using RowVector2d = RowVector<double, 2>;
using RowVector3d = RowVector<double, 3>;
using RowVectors2l = RowVectors<int64_t, 2>;
using RowVectors3l = RowVectors<int64_t, 3>;
using RowVectors4l = RowVectors<int64_t, 4>;
using RowVectors6l = RowVectors<int64_t, 6>;
using RowVectors2d = RowVectors<double, 2>;
using RowVectors3d = RowVectors<double, 3>;

using RowVectors3r = RowVectors<Rational, 3>;
using RowVectors2r = RowVectors<Rational, 2>;

using MatrixXl = MatrixX<int64_t>;
} // namespace wmtk
