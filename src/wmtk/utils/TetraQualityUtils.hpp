#pragma once

#include <Eigen/Core>
#include <optional>

namespace wmtk {

/**
 * Newton method for *linear* 1d parameterization.
 *
 * @param uv coordinates for the first vertex in the stack
 * @param stack with flattened 4x3 vertices positions, with the mover vertex always on the front.
 * This is the same convention with AMIPS_energy
 * @return Descend direction as computed from Newton method, (or gradient descent if Newton fails).
 */
double newton_method_from_stack(
    const double t,
    std::vector<std::array<double, 12>>& assembles,
    std::function<Eigen::Vector3d(const double t)> param,
    std::function<double(const std::array<double, 12>&)> compute_energy,
    std::function<double(const std::array<double, 12>&)> compute_jacobian,
    std::function<double(const std::array<double, 12>&)> compute_hessian);


/**
 * Newton method for *linear* 2d parameterization.
 *
 * @param uv coordinates for the first vertex in the stack
 * @param stack with flattened 4x3 vertices positions, with the mover vertex always on the front.
 * This is the same convention with AMIPS_energy
 * @return Descend direction as computed from Newton method, (or gradient descent if Newton fails).
 */
Eigen::Vector2d newton_method_from_stack(
    const Eigen::Vector2d& uv,
    std::vector<std::array<double, 12>>& assembles,
    std::function<Eigen::Vector3d(const Eigen::Vector2d& uv)> param,
    std::function<double(const std::array<double, 12>&)> compute_energy,
    std::function<void(const std::array<double, 12>&, Eigen::Vector2d&)> compute_jacobian,
    std::function<void(const std::array<double, 12>&, Eigen::Matrix2d&)> compute_hessian);

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

// TODO: Add documentation
Eigen::Vector3d try_project(
    const Eigen::Vector3d& point,
    const std::vector<std::array<double, 9>>& assembled_neighbor);
} // namespace wmtk