#pragma once

#include <array>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <vector>

#include <Eigen/Core>
#include <Rational.hpp>

namespace tetwild {

typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<apps::Rational, 3, 1> Vector3;

typedef Eigen::Vector2d Vector2d;
typedef Eigen::Matrix<apps::Rational, 2, 1> Vector2;

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

Vector3 to_rational(const Vector3d& p0);
Vector3d to_double(const Vector3& p0);

} // namespace tetwild
