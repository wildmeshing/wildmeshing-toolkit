#pragma once

#include <Eigen/Core>

#include <array>

namespace wmtk
{
	double AMIPS_energy(const std::array<double, 12> &T);
	void AMIPS_jacobian(const std::array<double, 12> &T, Eigen::Vector3d &result_0);
	void AMIPS_hessian(const std::array<double, 12> &T, Eigen::Matrix3d &result_0);
	template <typename rational> double AMIPS_energy_stable_p3(const std::array<double, 12>&);
} // namespace wmtk
