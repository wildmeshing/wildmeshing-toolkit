#pragma once

#include <predicates.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace wmtk {
template <int Axis, bool Invert>
struct AlignedHalfPlane
{
    static constexpr int axis = Axis;
    static constexpr bool invert = Invert;
    double coord = 0;
};

/// Stack-allocated matrix for storing the vertices of a 2D polygon when its maximum size is known
/// in advance.
template <int Size>
using SmallPolygon2d = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor, Size, 2>;

///
/// Compute signed area of the 2d polygon.
///
/// @param[in]  P     #n x 2 matrix of the input polygon vertices.
///
/// @return     Signed area of the polygon.
///
double polygon_signed_area(const Eigen::MatrixXd& P);

///
/// Clip a polygon by a half-plane.
/// https://github.com/alicevision/geogram/blob/cfbc0a5827d71d59f8bcf0369cc1731ef12f82ef/src/examples/graphics/demo_Delaunay2d/main.cpp#L677
///
/// @param[in]  P       #n x 2 matrix of the input polygon vertices.
/// @param[in]  q1      First endpoint of the clipping line.
/// @param[in]  q2      Second endpoint of the clipping line.
/// @param[out] result  Clipped polygon.
///
void clip_polygon_by_half_plane(
    const Eigen::MatrixXd& P,
    const Eigen::RowVector2d& q1,
    const Eigen::RowVector2d& q2,
    Eigen::MatrixXd& result);

///
/// Clip a polygon by an axis-aligned box.
///
/// @param[in]  triangle  2D triangle to clip.
/// @param[in]  box       Axis-aligned bbox to clip with.
///
/// @return     Clipped polygon.
///
SmallPolygon2d<7> clip_triangle_by_box(
    const SmallPolygon2d<3>& triangle,
    const Eigen::AlignedBox2d& box);

inline bool intersect_lines(
    const Eigen::RowVector2d& p1,
    const Eigen::RowVector2d& p2,
    const Eigen::RowVector2d& q1,
    const Eigen::RowVector2d& q2,
    Eigen::RowVector2d& result)
{
    Eigen::RowVector2d Vp = p2 - p1;
    Eigen::RowVector2d Vq = q2 - q1;
    Eigen::RowVector2d pq = q1 - p1;

    double a = Vp(0);
    double b = -Vq(0);
    double c = Vp(1);
    double d = -Vq(1);

    double delta = a * d - b * c;
    if (delta == 0.0) {
        return false;
    }

    double tp = (d * pq(0) - b * pq(1)) / delta;

    result << (1.0 - tp) * p1(0) + tp * p2(0), (1.0 - tp) * p1(1) + tp * p2(1);

    return true;
}

template <typename HalfPlaneType>
inline int point_is_in_aligned_half_plane(const Eigen::RowVector2d& p, HalfPlaneType half_plane)
{
    if (p[half_plane.axis] == half_plane.coord) {
        return 0;
    } else if (p[half_plane.axis] > half_plane.coord) {
        return half_plane.invert ? -1 : 1;
    } else {
        return half_plane.invert ? 1 : -1;
    }
}

template <typename HalfPlaneType>
inline double intersect_line_half_plane_in_barycentric(
    const Eigen::RowVector2d& p1,
    const Eigen::RowVector2d& p2,
    HalfPlaneType half_plane)
{
    constexpr int axis = half_plane.axis;

    if (p1[axis] == p2[axis]) {
        return -1.;
    }

    const double t = (half_plane.coord - p1[axis]) / (p2[axis] - p1[axis]);
    return t;
}

template <typename HalfPlaneType>
inline bool intersect_line_half_plane(
    const Eigen::RowVector2d& p1,
    const Eigen::RowVector2d& p2,
    HalfPlaneType half_plane,
    Eigen::RowVector2d& result)
{
    const double t = intersect_line_half_plane_in_barycentric<HalfPlaneType>(p1, p2, half_plane);
    if (t < 0) return false;
    result = (1.0 - t) * p1 + t * p2;

    return true;
}

inline int point_is_in_half_plane(
    const Eigen::RowVector2d& p,
    const Eigen::RowVector2d& q1,
    const Eigen::RowVector2d& q2)
{
    using Point = Eigen::Matrix<double, 2, 1>;
    Point a{p[0], p[1]};
    Point b{q1[0], q1[1]};
    Point c{q2[0], q2[1]};

    return (int)orient2d(b.data(), c.data(), a.data());
}

} // namespace wmtk
