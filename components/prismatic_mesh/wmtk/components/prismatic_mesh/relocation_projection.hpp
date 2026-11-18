#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace wmtk::components::prismatic_mesh::internal {

// Euclidean projection onto a ball intersected with a slab along a unit normal.
// The slab bounds lie in [-radius, radius] and define a nonempty intersection.
inline Eigen::Vector3d project_relocation_displacement(
    const Eigen::Vector3d& x,
    const Eigen::Vector3d& normal,
    double lower,
    double upper,
    double radius)
{
    Eigen::Vector3d ball = x;
    const double length = ball.stableNorm();
    if (length > radius) ball *= radius / length;
    const double ball_z = normal.dot(ball);
    if (ball_z >= lower && ball_z <= upper) return ball;
    const double z = std::clamp(ball_z, lower, upper);
    Eigen::Vector3d tangent = x - normal.dot(x) * normal;
    const double bound = std::sqrt(std::max(0., radius * radius - z * z));
    const double tangent_length = tangent.stableNorm();
    if (tangent_length > bound) tangent *= bound / tangent_length;
    return tangent + z * normal;
}

} // namespace wmtk::components::prismatic_mesh::internal
