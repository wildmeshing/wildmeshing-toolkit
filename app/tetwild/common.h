#pragma once

#include <array>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <vector>

#include <Eigen/Core>
#include <wmtk/utils/Rational.hpp>

namespace tetwild {

typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<wmtk::Rational, 3, 1> Vector3r;

typedef Eigen::Vector2d Vector2d;
typedef Eigen::Matrix<wmtk::Rational, 2, 1> Vector2r;

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

Vector3r to_rational(const Vector3d& p0);
Vector3d to_double(const Vector3r& p0);

} // namespace tetwild
