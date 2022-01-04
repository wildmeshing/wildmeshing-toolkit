//
// Created by Yixin Hu on 10/12/21.
//

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
#define MAX_ENERGY 1e50

typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<apps::Rational, 3, 1> Vector3;

typedef Eigen::Vector2d Vector2d;
typedef Eigen::Matrix<apps::Rational, 2, 1> Vector2;

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

Vector3 to_rational(const Vector3d& p0);

} // namespace tetwild
