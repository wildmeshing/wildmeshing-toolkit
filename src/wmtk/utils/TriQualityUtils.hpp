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
Eigen::Vector2d newton_method_from_stack_2d(
    std::vector<std::array<double, 6>>& stack,
    std::function<double(const std::array<double, 6>&)> energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&)> jacobian,
    std::function<void(const std::array<double, 6>&, Eigen::Matrix2d&)> hessian);

Eigen::Vector2d gradient_descent_from_stack_2d(
    std::vector<std::array<double, 6>>& stack,
    std::function<double(const std::array<double, 6>&)> energy,
    std::function<void(const std::array<double, 6>&, Eigen::Vector2d&)> jacobian);
/**
 * Reorders indices in a triangle such that v0 is on the front. Using the triangle symmetry to
 * preserve orientation. Assumes v0 is in the triangle.
 * @param conn
 * @param v0
 * @return std::array<size_t, 3>
 */
std::array<size_t, 3> orient_preserve_tri_reorder(const std::array<size_t, 3>& tri, size_t v0);

}
