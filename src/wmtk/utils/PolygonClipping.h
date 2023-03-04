#pragma once

#include <Eigen/Dense>
#include <vector>

namespace wmtk {

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

} // namespace wmtk
