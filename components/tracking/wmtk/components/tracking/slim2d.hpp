#pragma once

#include <Eigen/Core>

#include <array>
#include <vector>

namespace wmtk::components::tracking {

/**
 * @brief A triangle of a 2D layout and the shape it should keep.
 *
 * Its energy is the symmetric Dirichlet energy |J|^2 + |J^-1|^2 of the map J from the rest
 * triangle to its layout, times `weight`. A rest is given per element, not per vertex, so two
 * elements sharing a vertex may want different shapes of it -- the same outline vertex is in a
 * patch before an edit and in the patch after it.
 */
struct SlimElement
{
    std::array<int, 3> v;
    /// The inverse of the rest edge matrix [x1 - x0, x2 - x0].
    Eigen::Matrix2d rest_inv = Eigen::Matrix2d::Identity();
    double weight = 0;
};

/// An element resting in the shape of the 3D triangle (p0, p1, p2), laid flat, weighted by
/// `weight` times its area; zero weight if the triangle is degenerate.
SlimElement slim_element(
    const std::array<int, 3>& v,
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    double weight);

/// An element resting in the shape of the 2D triangle (q0, q1, q2), weighted by `weight`.
SlimElement slim_element(
    const std::array<int, 3>& v,
    const Eigen::Vector2d& q0,
    const Eigen::Vector2d& q1,
    const Eigen::Vector2d& q2,
    double weight);

/// The elements' energy in the layout `x`: infinite if one is not positively oriented, exactly.
double slim_energy(const std::vector<Eigen::Vector2d>& x, const std::vector<SlimElement>& elements);

/**
 * @brief SLIM (Rabinovich et al. 2017) on a 2D layout.
 *
 * Lowers the elements' energy by moving the vertices not `fixed`, `iterations` times or until
 * an iteration lowers it by less than 1e-4 of itself. The line search never lets an element turn
 * over or go flat -- it checks every step with the exact orient2d predicate -- so if every element
 * is positively oriented on entry, every one is on exit. Returns the final energy.
 */
double slim(
    std::vector<Eigen::Vector2d>& x,
    const std::vector<char>& fixed,
    const std::vector<SlimElement>& elements,
    int iterations);

} // namespace wmtk::components::tracking
