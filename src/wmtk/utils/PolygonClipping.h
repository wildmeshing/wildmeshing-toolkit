#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace wmtk {

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

} // namespace wmtk
