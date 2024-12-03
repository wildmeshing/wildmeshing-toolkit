#pragma once

#include <Eigen/Dense>
#include <utility>

// Function to find the tetrahedron containing the point and its barycentric coordinates
// V: Vertex matrix (n x 3)
// T: Tetrahedron index matrix (m x 4)
// p: Query point (3D)
// tolerance: Numerical tolerance for barycentric coordinate validation
std::pair<int, Eigen::Vector4d> findTetContainingPoint(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T,
    const Eigen::Vector3d& p,
    double tolerance = 1e-8);