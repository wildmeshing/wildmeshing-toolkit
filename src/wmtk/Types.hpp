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

using Vector2i = Vector<int, 2>;
using Vector3i = Vector<int, 3>;
using Vector4i = Vector<int, 4>;
using VectorXi = Vector<int, Eigen::Dynamic>;

inline Vector3r to_rational(const Vector3d& p0)
{
    Vector3r p(p0[0], p0[1], p0[2]);
    return p;
}

inline Vector3d to_double(const Vector3r& p0)
{
    Vector3d p(p0[0].to_double(), p0[1].to_double(), p0[2].to_double());
    return p;
}

} // namespace wmtk
