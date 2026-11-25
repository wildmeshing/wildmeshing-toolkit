#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh {
struct RodJointRadiusSample
{
    double residual = 0;
    Vector3d gradient = Vector3d::Zero(); // Derivative in rod-radius coordinates.
};

// Smooth union of incident capsules. A single branch gives its usual radius;
// the small blend fills the crease where multiple branch tubes meet. Subtract
// the minimum before exponentiation to keep distant queries numerically stable.
inline RodJointRadiusSample rod_joint_radius_sample(
    const Vector3d& p,
    const std::vector<std::array<Vector3d, 2>>& segments,
    double radius,
    double blend)
{
    std::vector<double> distances;
    std::vector<Vector3d> gradients;
    for (const auto& edge : segments) {
        const Vector3d d = edge[1] - edge[0];
        const Vector3d foot =
            edge[0] + std::clamp((p - edge[0]).dot(d) / d.squaredNorm(), 0., 1.) * d;
        const Vector3d radial = (p - foot) / radius;
        const double length = radial.norm();
        distances.push_back(length);
        gradients.push_back(length > 1e-12 ? Vector3d(radial / length) : Vector3d::Zero());
    }
    if (distances.empty()) throw std::runtime_error("joint radius has no incident segments");
    const auto minimum = std::min_element(distances.begin(), distances.end());
    if (!(blend > 0)) {
        const size_t i = minimum - distances.begin();
        return {distances[i] - 1, gradients[i]};
    }
    double sum = 0;
    Vector3d gradient = Vector3d::Zero();
    for (size_t i = 0; i < distances.size(); ++i) {
        const double weight = std::exp(-(distances[i] - *minimum) / blend);
        sum += weight;
        gradient += weight * gradients[i];
    }
    return {*minimum - blend * std::log(sum) - 1, gradient / sum};
}
} // namespace wmtk::components::prismatic_mesh
