#pragma once
#include <Eigen/Dense>
#include <algorithm>

namespace wmtk::components::topological_offset::exact_simplex_distance {

/**
 * @brief Closest-point distances between two simplices, in double arithmetic and never sampled.
 *
 * The refined march's bound B is the smallest distance between the input complex and any far
 * simplex of a band cell (see TopoOffsetTetMesh::refined_marching_bound()). Sampling a simplex and
 * taking the smallest sample overestimates that distance, and B is what march_distance is a
 * fraction of: an overestimate puts the front outside the band, which is the one thing
 * march_distance exists to prevent. So every pair below is solved in closed form.
 *
 * Everything is squared distance -- the caller takes one square root at the end. The vector type
 * is templated because 2D and 3D need the same point-point, point-segment and segment-segment
 * formulas; only point-triangle and the pairs built on it are 3D.
 */

template <typename Vec>
inline double point_point_sq(const Vec& p, const Vec& q)
{
    return (p - q).squaredNorm();
}

/// Ericson, Real-Time Collision Detection 5.1.2: the parameter is clamped to the segment, so a
/// degenerate segment (a == b) falls back to the point-point distance.
template <typename Vec>
inline double point_segment_sq(const Vec& p, const Vec& a, const Vec& b)
{
    const Vec ab = b - a;
    const double len2 = ab.squaredNorm();
    if (!(len2 > 0.)) return (p - a).squaredNorm();
    const double t = std::clamp((p - a).dot(ab) / len2, 0., 1.);
    return (p - (a + t * ab)).squaredNorm();
}

/// Ericson 5.1.5, the barycentric-region form: the closest point of triangle (a, b, c) to p is
/// found by testing the six Voronoi regions of the triangle's vertices and edges before the face.
inline double point_triangle_sq(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
    const Eigen::Vector3d ab = b - a, ac = c - a, ap = p - a;
    const double d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0. && d2 <= 0.) return (p - a).squaredNorm();

    const Eigen::Vector3d bp = p - b;
    const double d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0. && d4 <= d3) return (p - b).squaredNorm();

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0. && d1 >= 0. && d3 <= 0.) {
        const double den = d1 - d3;
        const double v = den > 0. ? d1 / den : 0.;
        return (p - (a + v * ab)).squaredNorm();
    }

    const Eigen::Vector3d cp = p - c;
    const double d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0. && d5 <= d6) return (p - c).squaredNorm();

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0. && d2 >= 0. && d6 <= 0.) {
        const double den = d2 - d6;
        const double w = den > 0. ? d2 / den : 0.;
        return (p - (a + w * ac)).squaredNorm();
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0.) {
        const double den = (d4 - d3) + (d5 - d6);
        const double w = den > 0. ? (d4 - d3) / den : 0.;
        return (p - (b + w * (c - b))).squaredNorm();
    }

    const double den = va + vb + vc;
    if (!(den > 0.)) { // degenerate triangle: its three edges are the whole of it
        return std::min(
            {point_segment_sq(p, a, b), point_segment_sq(p, b, c), point_segment_sq(p, c, a)});
    }
    const double v = vb / den, w = vc / den;
    return (p - (a + v * ab + w * ac)).squaredNorm();
}

/// Ericson 5.1.9: closest points of segments (p1, q1) and (p2, q2), both parameters clamped, with
/// the parallel and degenerate cases taken separately.
template <typename Vec>
inline double segment_segment_sq(const Vec& p1, const Vec& q1, const Vec& p2, const Vec& q2)
{
    const Vec d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const double a = d1.squaredNorm(), e = d2.squaredNorm(), f = d2.dot(r);
    if (!(a > 0.) && !(e > 0.)) return r.squaredNorm();
    if (!(a > 0.)) return point_segment_sq(p1, p2, q2);
    if (!(e > 0.)) return point_segment_sq(p2, p1, q1);

    const double c = d1.dot(r), b = d1.dot(d2);
    const double den = a * e - b * b;
    double s = den > 0. ? std::clamp((b * f - c * e) / den, 0., 1.) : 0.; // parallel: s = 0
    double t = (b * s + f) / e;
    if (t < 0.) {
        t = 0.;
        s = std::clamp(-c / a, 0., 1.);
    } else if (t > 1.) {
        t = 1.;
        s = std::clamp((b - c) / a, 0., 1.);
    }
    return ((p1 + s * d1) - (p2 + t * d2)).squaredNorm();
}

/// The two simplices do not intersect (the input complex and a far simplex of a band cell are
/// disjoint), so the closest pair involves a boundary feature of at least one of them: either an
/// endpoint of the segment against the triangle, or the segment against one of the triangle's
/// edges (which also covers a triangle vertex against the segment's interior).
inline double segment_triangle_sq(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& q,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
    return std::min(
        {point_triangle_sq(p, a, b, c),
         point_triangle_sq(q, a, b, c),
         segment_segment_sq(p, q, a, b),
         segment_segment_sq(p, q, b, c),
         segment_segment_sq(p, q, c, a)});
}

/// Same argument one dimension up: six vertex-triangle pairs and nine edge-edge pairs.
inline double triangle_triangle_sq(
    const Eigen::Vector3d& a0,
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& a2,
    const Eigen::Vector3d& b0,
    const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2)
{
    double best = std::min(
        {point_triangle_sq(a0, b0, b1, b2),
         point_triangle_sq(a1, b0, b1, b2),
         point_triangle_sq(a2, b0, b1, b2),
         point_triangle_sq(b0, a0, a1, a2),
         point_triangle_sq(b1, a0, a1, a2),
         point_triangle_sq(b2, a0, a1, a2)});
    const Eigen::Vector3d A[3][2] = {{a0, a1}, {a1, a2}, {a2, a0}};
    const Eigen::Vector3d B[3][2] = {{b0, b1}, {b1, b2}, {b2, b0}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            best = std::min(best, segment_segment_sq(A[i][0], A[i][1], B[j][0], B[j][1]));
        }
    }
    return best;
}

} // namespace wmtk::components::topological_offset::exact_simplex_distance
