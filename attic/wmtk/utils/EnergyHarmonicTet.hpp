#pragma once

#include <Eigen/Core>

#include <array>

namespace wmtk {
double harmonic_tet_energy(const std::array<double, 12>& T);
void harmonic_tet_jacobian(const std::array<double, 12>& T, Eigen::Vector3d& result_0);

/**
 * @brief Harmonic Triangulation energy: trace of Laplacian operator
 *
 */
double harmonic_energy(const Eigen::MatrixXd& verts);

} // namespace wmtk
