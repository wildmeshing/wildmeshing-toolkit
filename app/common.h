//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <array>
#include <queue>

#include <Eigen/Core>
#include <wmtk/Logger.hpp>

namespace tetwild
{
	using std::cin;
	using std::cout;
	using std::endl;

#define MAX_ENERGY 1e50

	typedef Eigen::Vector3d Vector3f; // float?
	typedef Eigen::Vector3d Vector3;

	typedef double Scalar;
	typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

} // namespace tetwild
