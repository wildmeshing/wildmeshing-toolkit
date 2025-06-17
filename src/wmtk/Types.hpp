#pragma once

#include <Eigen/Core>
#include <wmtk/utils/Rational.hpp>


namespace wmtk {

template <typename T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

using MatrixXd = MatrixX<double>;
using MatrixXi = MatrixX<int>;

template <typename T, int R>
using Vector = Eigen::Matrix<T, R, 1>;
template <typename T>
using VectorX = Vector<T, Eigen::Dynamic>;

using Vector2d = Vector<double, 2>;
using Vector3d = Vector<double, 3>;
using VectorXd = Vector<double, Eigen::Dynamic>;

using Vector2r = Vector<Rational, 2>;
using Vector3r = Vector<Rational, 3>;

} // namespace wmtk
