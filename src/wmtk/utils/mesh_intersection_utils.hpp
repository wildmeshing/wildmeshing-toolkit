#pragma once

#include <Eigen/Core>
#include <vector>

namespace wmtk::utils {

/**
 * @brief Check if a 2D triangle mesh has self-intersections by checking boundary loop
 * For 2D planar meshes, checking boundary loop self-intersection is sufficient
 * @param V Vertex matrix (n x 2)
 * @param F Face matrix (m x 3)
 * @param eps Numerical tolerance for intersection test
 * @return True if mesh has self-intersections, false otherwise
 */
bool hasSelfIntersection2D(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    double eps = 1e-10);

/**
 * @brief Check if a 3D surface triangle mesh has self-intersections
 * For 3D surface meshes, need to check all triangle pairs for intersection
 * @param V Vertex matrix (n x 3) 
 * @param F Face matrix (m x 3)
 * @param eps Numerical tolerance
 * @return True if mesh has self-intersections, false otherwise
 */
bool hasSelfIntersection3D(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    double eps = 1e-10);

/**
 * @brief Check if two 2D line segments intersect (used internally)
 * @param p1, q1 First line segment endpoints
 * @param p2, q2 Second line segment endpoints  
 * @param eps Numerical tolerance for intersection test
 * @return True if segments intersect, false otherwise
 */
bool doSegmentsIntersect2D(
    const Eigen::RowVector2d& p1,
    const Eigen::RowVector2d& q1, 
    const Eigen::RowVector2d& p2,
    const Eigen::RowVector2d& q2,
    double eps = 1e-10);

/**
 * @brief Check if two 3D triangles intersect (used internally)
 * @param tri1 First triangle vertices (3x3 matrix, each row is a vertex)
 * @param tri2 Second triangle vertices (3x3 matrix, each row is a vertex)
 * @param eps Numerical tolerance for intersection test
 * @return True if triangles intersect, false otherwise
 */
bool doTrianglesIntersect3D(
    const Eigen::Matrix<double, 3, 3>& tri1,
    const Eigen::Matrix<double, 3, 3>& tri2,
    double eps = 1e-10);

} // namespace wmtk::utils