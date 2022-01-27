#pragma once

#include <Eigen/Core>
#include <optional>

namespace wmtk {

/**
 * Newton method or gradient descent.
 *
 * @param stack with flattened 4x3 vertices positions, with the mover vertex always on the front.
 * This is the same convention with AMIPS_energy
 * @return Descend direction as computed from Newton method, (or gradient descent if Newton fails).
 */
Eigen::Vector3d newton_method_from_stack(
    std::vector<std::array<double, 12>>& stack,
    std::function<double(const std::array<double, 12>&)> energy,
    std::function<void(const std::array<double, 12>&, Eigen::Vector3d&)> jacobian,
    std::function<void(const std::array<double, 12>&, Eigen::Matrix3d&)> hessian);

Eigen::Vector3d gradient_descent_from_stack(
    std::vector<std::array<double, 12>>& stack,
    std::function<double(const std::array<double, 12>&)> energy,
    std::function<void(const std::array<double, 12>&, Eigen::Vector3d&)> jacobian);
/**
 * Reorders indices in a tetrahedron such that v0 is on the front. Using the tetra symmetry to
 * preserve orientation. Assumes v0 in tetra.
 * @param conn
 * @param v0
 * @return std::array<size_t, 4>
 */
std::array<size_t, 4> orient_preserve_tet_reorder(const std::array<size_t, 4>& tetra, size_t v0);

/**
 * @brief Harmonic Triangulation energy: trace of Laplacian operator
 *
 */
double harmonic_energy(const Eigen::MatrixXd& verts);

std::optional<Eigen::Vector3d> try_project(
    const Eigen::Vector3d& point,
    const std::vector<std::array<double, 12>>& assembled_neighbor);
} // namespace wmtk