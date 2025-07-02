#pragma once

#include <Eigen/Dense>
#include <utility>

#include <wmtk/utils/Rational.hpp>
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

// Rational version of findTetContainingPoint for higher precision
// V: Vertex matrix (n x 3) with rational coordinates
// T: Tetrahedron index matrix (m x 4)
// p: Query point (3D) with rational coordinates
std::pair<int, Eigen::Matrix<wmtk::Rational, 4, 1>> findTetContainingPointRational(
    const Eigen::Matrix<wmtk::Rational, Eigen::Dynamic, 3>& V,
    const Eigen::MatrixXi& T,
    const Eigen::Matrix<wmtk::Rational, 3, 1>& p);

// Numerically stable version using wmtk::orient3d for point-in-tetrahedron testing
// V: Vertex matrix (n x 3)
// T: Tetrahedron index matrix (m x 4)
// p: Query point (3D)
std::pair<int, Eigen::Vector4d> findTetContainingPointOrient3d(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T,
    const Eigen::Vector3d& p);