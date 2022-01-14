#pragma once

#include <Eigen/Core>

#include <array>

namespace wmtk {
double harmonic_tet_energy(const std::array<double, 12>& T);
void harmonic_tet_jacobian(const std::array<double, 12>& T, Eigen::Vector3d& result_0);
} // namespace wmtk
