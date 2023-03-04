#pragma once

#include "Quadrature.h"

namespace wmtk {

class ClippedQuadrature
{
public:
    using TriangleVertices = Eigen::Matrix<double, 3, 2, Eigen::RowMajor>;
    using PolygonVertices = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;

public:
    ///
    /// Computes quadrature points and weights for a triangle clipped by an axis-aligned box.
    ///
    /// @param[in]  order     Requested quadrature order.
    /// @param[in]  triangle  Vertices of the triangle to be clipped.
    /// @param[in]  box       Axis-aligned box to clip with.
    /// @param[out] quadr     Output quadrature points and weights.
    ///
    static void clipped_triangle_quad_quadrature(
        const int order,
        const TriangleVertices& triangle,
        const Eigen::AlignedBox2d& box,
        Quadrature& quadr);

    ///
    /// Computes quadrature points and weights for a triangle clipped by a convex polygon.
    ///
    /// @param[in]  order     Requested quadrature order.
    /// @param[in]  triangle  Vertices of the triangle to be clipped.
    /// @param[in]  polygon   Vertices of the convex polygon to clip with.
    /// @param[out] quadr     Output quadrature points and weights.
    ///
    static void clipped_triangle_polygon_quadrature(
        const int order,
        const TriangleVertices& triangle,
        const PolygonVertices& polygon,
        Quadrature& quadr);
};

} // namespace wmtk
