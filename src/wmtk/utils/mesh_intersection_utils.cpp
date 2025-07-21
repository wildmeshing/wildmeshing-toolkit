#include "mesh_intersection_utils.hpp"
#include <igl/boundary_loop.h>
#include <cmath>
#include <algorithm>

namespace wmtk::utils {

namespace {
    // Helper function for point-on-segment test
    bool onSegment(const Eigen::RowVector2d& p, const Eigen::RowVector2d& q, const Eigen::RowVector2d& r, double eps) {
        return q[0] <= std::max(p[0], r[0]) + eps &&
               q[0] >= std::min(p[0], r[0]) - eps &&
               q[1] <= std::max(p[1], r[1]) + eps &&
               q[1] >= std::min(p[1], r[1]) - eps;
    }

    // Helper function for orientation test
    int orientation(const Eigen::RowVector2d& p, const Eigen::RowVector2d& q, const Eigen::RowVector2d& r, double eps) {
        double val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
        if (std::abs(val) < eps) return 0; // collinear
        return (val > 0) ? 1 : 2; // clock or counterclock wise
    }

    // Helper function for 3D triangle-triangle intersection using Möller-Trumbore algorithm
    bool triangleTriangleIntersect3D(
        const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2,
        const Eigen::Vector3d& u0, const Eigen::Vector3d& u1, const Eigen::Vector3d& u2,
        double eps) {
        
        // Compute plane equation of triangle(u0,u1,u2)
        Eigen::Vector3d e1 = u1 - u0;
        Eigen::Vector3d e2 = u2 - u0;
        Eigen::Vector3d n2 = e1.cross(e2);
        
        // Check if triangles are coplanar
        double d2 = -n2.dot(u0);
        double du0 = n2.dot(v0) + d2;
        double du1 = n2.dot(v1) + d2;
        double du2 = n2.dot(v2) + d2;
        
        if (std::abs(du0) < eps) du0 = 0.0;
        if (std::abs(du1) < eps) du1 = 0.0;
        if (std::abs(du2) < eps) du2 = 0.0;
        
        double du0du1 = du0 * du1;
        double du0du2 = du0 * du2;
        
        if (du0du1 > 0.0 && du0du2 > 0.0) return false; // same sign on all points, no intersection
        
        // Compute plane equation of triangle(v0,v1,v2)
        Eigen::Vector3d e3 = v1 - v0;
        Eigen::Vector3d e4 = v2 - v0;
        Eigen::Vector3d n1 = e3.cross(e4);
        
        double d1 = -n1.dot(v0);
        double dv0 = n1.dot(u0) + d1;
        double dv1 = n1.dot(u1) + d1;
        double dv2 = n1.dot(u2) + d1;
        
        if (std::abs(dv0) < eps) dv0 = 0.0;
        if (std::abs(dv1) < eps) dv1 = 0.0;
        if (std::abs(dv2) < eps) dv2 = 0.0;
        
        double dv0dv1 = dv0 * dv1;
        double dv0dv2 = dv0 * dv2;
        
        if (dv0dv1 > 0.0 && dv0dv2 > 0.0) return false; // same sign on all points, no intersection
        
        // If we reach here, triangles intersect
        return true;
    }
}

bool doSegmentsIntersect2D(
    const Eigen::RowVector2d& p1,
    const Eigen::RowVector2d& q1, 
    const Eigen::RowVector2d& p2,
    const Eigen::RowVector2d& q2,
    double eps) {
    
    int o1 = orientation(p1, q1, p2, eps);
    int o2 = orientation(p1, q1, q2, eps);
    int o3 = orientation(p2, q2, p1, eps);
    int o4 = orientation(p2, q2, q1, eps);

    // General case
    if (o1 != o2 && o3 != o4) return true;

    // Special Cases - collinear points
    if (o1 == 0 && onSegment(p1, p2, q1, eps)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1, eps)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2, eps)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2, eps)) return true;

    return false; // No intersection
}

bool hasSelfIntersection2D(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    double eps) {
    
    // Extract the boundary loop
    Eigen::VectorXi loop;
    igl::boundary_loop(F, loop);
    
    int n = loop.size();
    if (n < 3) return false; // Need at least 3 points for a loop
    
    // Check all non-adjacent boundary edge pairs for intersection
    for (int i = 0; i < n; ++i) {
        Eigen::RowVector2d p1 = V.row(loop[i]);
        Eigen::RowVector2d q1 = V.row(loop[(i + 1) % n]);
        
        for (int j = i + 2; j < n; ++j) {
            // Skip adjacent edges (they share vertices)
            if (j != (i + 1) % n && i != (j + 1) % n) {
                Eigen::RowVector2d p2 = V.row(loop[j]);
                Eigen::RowVector2d q2 = V.row(loop[(j + 1) % n]);
                
                if (doSegmentsIntersect2D(p1, q1, p2, q2, eps)) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

bool doTrianglesIntersect3D(
    const Eigen::Matrix<double, 3, 3>& tri1,
    const Eigen::Matrix<double, 3, 3>& tri2,
    double eps) {
    
    return triangleTriangleIntersect3D(
        tri1.row(0), tri1.row(1), tri1.row(2),
        tri2.row(0), tri2.row(1), tri2.row(2),
        eps);
}

bool hasSelfIntersection3D(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    double eps) {
    
    int num_faces = F.rows();
    
    // Check all pairs of triangles for intersection
    for (int i = 0; i < num_faces; ++i) {
        for (int j = i + 1; j < num_faces; ++j) {
            // Skip adjacent triangles (they share vertices or edges)
            bool adjacent = false;
            for (int vi = 0; vi < 3; ++vi) {
                for (int vj = 0; vj < 3; ++vj) {
                    if (F(i, vi) == F(j, vj)) {
                        adjacent = true;
                        break;
                    }
                }
                if (adjacent) break;
            }
            
            if (!adjacent) {
                Eigen::Matrix<double, 3, 3> tri1, tri2;
                tri1.row(0) = V.row(F(i, 0));
                tri1.row(1) = V.row(F(i, 1));
                tri1.row(2) = V.row(F(i, 2));
                
                tri2.row(0) = V.row(F(j, 0));
                tri2.row(1) = V.row(F(j, 1));
                tri2.row(2) = V.row(F(j, 2));
                
                if (doTrianglesIntersect3D(tri1, tri2, eps)) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

} // namespace wmtk::utils