#pragma once
#include <array>
#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh {
struct PrismJacobianValue
{
    double determinant;
    std::array<Vector3d, 6> gradients; // derivatives with respect to physical corner coordinates
};
struct PrismJacobianMinimum
{
    double determinant;
    std::array<double, 3> edge_values;
    std::array<Vector3d, 3> edge_points;
};
// Analytic shape derivatives, including at singular J (no matrix inversion).
PrismJacobianValue
prism_jacobian(const MatrixXd& vertices, const HybridCell& prism, const Vector3d& reference_point);
// det(J) is affine in (r,s) and quadratic in t. Its minimum occurs on one of the
// three column edges: examine the endpoints and any interior quadratic minimum.
PrismJacobianMinimum minimum_prism_jacobian(const MatrixXd& vertices, const HybridCell& prism);
std::vector<Vector3d> prism_jacobian_sample_points();
} // namespace wmtk::components::prismatic_mesh
