#pragma once

#include <Eigen/Core>

#include <array>
#include <functional>

namespace wmtk::utils {

/**
 * @file
 * Conservative tests of a function over a whole cell (a triangle in 2D, a tet in 3D) by ball
 * refinement. If f is 1-Lipschitz (a distance function, for one), a ball (circle, sphere) of
 * radius rho around c holds only values within rho of f(c), so one evaluation decides the whole
 * ball when f(c) is more than rho from the level. Otherwise the ball is refined: the ball through
 * the corners of the cell's bounding square (cube) is replaced by the balls of its square's 2^dim
 * sub-squares that overlap the cell, and so on down.
 *
 * Moved here from topological_offset's Circle.hpp and Sphere.hpp, and tight_seal_geom's
 * certify_on_cell, which used the same refinement.
 */

/// A ball of the refinement: a circle (D = 2) or a sphere (D = 3).
template <int D>
struct RefinementBall
{
    using Vec = Eigen::Matrix<double, D, 1>;
    using Cell = std::array<Vec, D + 1>;

    Vec center = Vec::Zero();
    double radius = 0;

    /// The ball through the corners of the cell's bounding square (cube, see fit_box): centred on
    /// it, of radius side x sqrt(D) / 2.
    static RefinementBall bounding(const Cell& cell);

    /// The balls of the square's (cube's) 2^D sub-squares: radius / 2, centred radius / (2 sqrt(D))
    /// away along each axis. Child s is on the + side of axis k where bit k of s is set, so the
    /// first is the all-minus one.
    std::array<RefinementBall, (1 << D)> refined() const;

    /// Whether the open ball meets the closed cell: the cell is closer to the centre than radius.
    bool overlaps(const Cell& cell) const;
};

using RefinementCircle = RefinementBall<2>;
using RefinementSphere = RefinementBall<3>;

/// The cell's bounding square (cube): the centre of its bounding box, and as side the largest of
/// the box's spans.
template <int D>
void fit_box(
    const std::array<Eigen::Matrix<double, D, 1>, D + 1>& cell,
    Eigen::Matrix<double, D, 1>& center,
    double& side);

/// The distance from p to a closed cell: a triangle in 2D, a tet in 3D; 0 inside it.
double point_cell_distance(const Eigen::Vector2d& p, const std::array<Eigen::Vector2d, 3>& cell);
double point_cell_distance(const Eigen::Vector3d& p, const std::array<Eigen::Vector3d, 4>& cell);

/// The distance from p to the closed triangle (a, b, c) in 3D.
double point_triangle_distance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c);

/// The default relative margin of certify_on_cell's tests.
inline constexpr double certify_default_margin = 1e-12;

/**
 * @brief Certifies that a 1-Lipschitz function f stays on one side of `level` over a whole cell,
 * conservatively: true only if proven.
 *
 * at_most: max over the cell of f <= level; otherwise: min over the cell of f > level. f is any
 * 1-Lipschitz function, typically the distance to some set. A ball whose values are all on the
 * right side, by the relative margin `margin` (so that rounding can only refute), is certified;
 * the cell is refuted if a corner is not (`corner_values`: f at the corners, with the margin), or
 * if a ball's centre lies in the cell with f on the wrong side; any other ball is refined into its
 * children that overlap the cell, and refutes the cell once `max_depth` levels down. NaN refutes.
 */
bool certify_on_cell(
    const std::array<Eigen::Vector2d, 3>& cell,
    const std::array<double, 3>& corner_values,
    const std::function<double(const Eigen::Vector2d&)>& f,
    double level,
    bool at_most,
    int max_depth,
    double margin = certify_default_margin);
bool certify_on_cell(
    const std::array<Eigen::Vector3d, 4>& cell,
    const std::array<double, 4>& corner_values,
    const std::function<double(const Eigen::Vector3d&)>& f,
    double level,
    bool at_most,
    int max_depth,
    double margin = certify_default_margin);

} // namespace wmtk::utils
