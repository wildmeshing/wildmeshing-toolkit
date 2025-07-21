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

    // Helper function to test if point is inside triangle (2D projection)
    bool pointInTriangle2D(const Eigen::Vector2d& p, 
                          const Eigen::Vector2d& a, 
                          const Eigen::Vector2d& b, 
                          const Eigen::Vector2d& c,
                          double eps) {
        // Use barycentric coordinates
        Eigen::Vector2d v0 = c - a;
        Eigen::Vector2d v1 = b - a;
        Eigen::Vector2d v2 = p - a;
        
        double dot00 = v0.dot(v0);
        double dot01 = v0.dot(v1);
        double dot02 = v0.dot(v2);
        double dot11 = v1.dot(v1);
        double dot12 = v1.dot(v2);
        
        double invDenom = dot00 * dot11 - dot01 * dot01;
        if (std::abs(invDenom) < eps) return false; // degenerate triangle
        
        invDenom = 1.0 / invDenom;
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        
        return (u >= -eps) && (v >= -eps) && (u + v <= 1.0 + eps);
    }
    
    // Helper function to test if two 2D segments intersect
    bool segments2DIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1,
                           const Eigen::Vector2d& p2, const Eigen::Vector2d& q2,
                           double eps) {
        Eigen::Vector2d d1 = q1 - p1;
        Eigen::Vector2d d2 = q2 - p2;
        Eigen::Vector2d r = p1 - p2;
        
        double cross_d1_d2 = d1.x() * d2.y() - d1.y() * d2.x();
        if (std::abs(cross_d1_d2) < eps) return false; // parallel
        
        double t = (r.x() * d2.y() - r.y() * d2.x()) / cross_d1_d2;
        double u = (r.x() * d1.y() - r.y() * d1.x()) / cross_d1_d2;
        
        return (t >= -eps && t <= 1.0 + eps && u >= -eps && u <= 1.0 + eps);
    }
    
    // Helper function to handle coplanar triangle intersection
    bool coplanarTrianglesIntersect(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2,
                                   const Eigen::Vector3d& u0, const Eigen::Vector3d& u1, const Eigen::Vector3d& u2,
                                   const Eigen::Vector3d& normal, double eps) {
        // Project triangles onto the plane with the largest normal component
        int axis = 0;
        if (std::abs(normal.y()) > std::abs(normal.x())) axis = 1;
        if (std::abs(normal.z()) > std::abs(normal[axis])) axis = 2;
        
        auto project = [axis](const Eigen::Vector3d& p) -> Eigen::Vector2d {
            if (axis == 0) return Eigen::Vector2d(p.y(), p.z());
            if (axis == 1) return Eigen::Vector2d(p.x(), p.z());
            return Eigen::Vector2d(p.x(), p.y());
        };
        
        Eigen::Vector2d v0_2d = project(v0), v1_2d = project(v1), v2_2d = project(v2);
        Eigen::Vector2d u0_2d = project(u0), u1_2d = project(u1), u2_2d = project(u2);
        
        // Check if any vertex of one triangle is inside the other
        if (pointInTriangle2D(v0_2d, u0_2d, u1_2d, u2_2d, eps) ||
            pointInTriangle2D(v1_2d, u0_2d, u1_2d, u2_2d, eps) ||
            pointInTriangle2D(v2_2d, u0_2d, u1_2d, u2_2d, eps) ||
            pointInTriangle2D(u0_2d, v0_2d, v1_2d, v2_2d, eps) ||
            pointInTriangle2D(u1_2d, v0_2d, v1_2d, v2_2d, eps) ||
            pointInTriangle2D(u2_2d, v0_2d, v1_2d, v2_2d, eps)) {
            return true;
        }
        
        // Check if any edge of one triangle intersects any edge of the other
        Eigen::Vector2d tri1_edges[3][2] = {{v0_2d, v1_2d}, {v1_2d, v2_2d}, {v2_2d, v0_2d}};
        Eigen::Vector2d tri2_edges[3][2] = {{u0_2d, u1_2d}, {u1_2d, u2_2d}, {u2_2d, u0_2d}};
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (segments2DIntersect(tri1_edges[i][0], tri1_edges[i][1],
                                      tri2_edges[j][0], tri2_edges[j][1], eps)) {
                    return true;
                }
            }
        }
        
        return false;
    }

    // Robust 3D triangle-triangle intersection using Möller's 1997 algorithm
    bool triangleTriangleIntersect3D(
        const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2,
        const Eigen::Vector3d& u0, const Eigen::Vector3d& u1, const Eigen::Vector3d& u2,
        double eps) {
        
        // Compute plane equation of triangle 1 (v0,v1,v2)
        Eigen::Vector3d e1 = v1 - v0;
        Eigen::Vector3d e2 = v2 - v0;
        Eigen::Vector3d n1 = e1.cross(e2);
        
        // Check if triangle 1 is degenerate
        if (n1.norm() < eps) return false;
        
        double d1 = -n1.dot(v0);
        
        // Test triangle 2 vertices against triangle 1 plane
        double du0 = n1.dot(u0) + d1;
        double du1 = n1.dot(u1) + d1;
        double du2 = n1.dot(u2) + d1;
        
        // Apply epsilon tolerance
        if (std::abs(du0) < eps) du0 = 0.0;
        if (std::abs(du1) < eps) du1 = 0.0;
        if (std::abs(du2) < eps) du2 = 0.0;
        
        double du0du1 = du0 * du1;
        double du0du2 = du0 * du2;
        
        // If all points of triangle 2 are on the same side of plane 1, no intersection
        if (du0du1 > 0.0 && du0du2 > 0.0) return false;
        
        // Compute plane equation of triangle 2 (u0,u1,u2)
        Eigen::Vector3d e3 = u1 - u0;
        Eigen::Vector3d e4 = u2 - u0;
        Eigen::Vector3d n2 = e3.cross(e4);
        
        // Check if triangle 2 is degenerate
        if (n2.norm() < eps) return false;
        
        double d2 = -n2.dot(u0);
        
        // Test triangle 1 vertices against triangle 2 plane
        double dv0 = n2.dot(v0) + d2;
        double dv1 = n2.dot(v1) + d2;
        double dv2 = n2.dot(v2) + d2;
        
        // Apply epsilon tolerance
        if (std::abs(dv0) < eps) dv0 = 0.0;
        if (std::abs(dv1) < eps) dv1 = 0.0;
        if (std::abs(dv2) < eps) dv2 = 0.0;
        
        double dv0dv1 = dv0 * dv1;
        double dv0dv2 = dv0 * dv2;
        
        // If all points of triangle 1 are on the same side of plane 2, no intersection
        if (dv0dv1 > 0.0 && dv0dv2 > 0.0) return false;
        
        // Check if triangles are coplanar
        if (std::abs(du0) < eps && std::abs(du1) < eps && std::abs(du2) < eps) {
            return coplanarTrianglesIntersect(v0, v1, v2, u0, u1, u2, n1, eps);
        }
        
        // Compute intersection line direction
        Eigen::Vector3d direction = n1.cross(n2);
        
        // Check if planes are parallel (no intersection line)
        if (direction.norm() < eps) return false;
        
        // Find the largest component of direction to avoid division by small numbers
        int maxc = 0;
        if (std::abs(direction.y()) > std::abs(direction.x())) maxc = 1;
        if (std::abs(direction.z()) > std::abs(direction[maxc])) maxc = 2;
        
        // Project triangles onto intersection line and compute intervals
        auto computeInterval = [&](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                                   double dp0, double dp1, double dp2) -> std::pair<double, double> {
            double t0, t1;
            
            if (dp0 * dp1 > 0.0) {
                // dp0 and dp1 have same sign, dp2 is different
                double proj0 = p0[maxc];
                double proj2 = p2[maxc];
                double alpha = dp2 / (dp2 - dp0);
                t0 = proj2 + alpha * (proj0 - proj2);
                
                double proj1 = p1[maxc];
                alpha = dp2 / (dp2 - dp1);
                t1 = proj2 + alpha * (proj1 - proj2);
            } else if (dp0 * dp2 > 0.0) {
                // dp0 and dp2 have same sign, dp1 is different
                double proj0 = p0[maxc];
                double proj1 = p1[maxc];
                double alpha = dp1 / (dp1 - dp0);
                t0 = proj1 + alpha * (proj0 - proj1);
                
                double proj2 = p2[maxc];
                alpha = dp1 / (dp1 - dp2);
                t1 = proj1 + alpha * (proj2 - proj1);
            } else {
                // dp1 and dp2 have same sign, dp0 is different
                double proj1 = p1[maxc];
                double proj0 = p0[maxc];
                double alpha = dp0 / (dp0 - dp1);
                t0 = proj0 + alpha * (proj1 - proj0);
                
                double proj2 = p2[maxc];
                alpha = dp0 / (dp0 - dp2);
                t1 = proj0 + alpha * (proj2 - proj0);
            }
            
            if (t0 > t1) std::swap(t0, t1);
            return {t0, t1};
        };
        
        auto [t1_min, t1_max] = computeInterval(v0, v1, v2, dv0, dv1, dv2);
        auto [t2_min, t2_max] = computeInterval(u0, u1, u2, du0, du1, du2);
        
        // Check if intervals overlap (with epsilon tolerance)
        return (t1_max >= t2_min - eps) && (t2_max >= t1_min - eps);
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