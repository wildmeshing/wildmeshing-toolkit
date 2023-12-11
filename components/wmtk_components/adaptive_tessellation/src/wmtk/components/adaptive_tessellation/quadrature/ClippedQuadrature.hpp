#pragma once

#include "Quadrature.hpp"

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
    /// @param[in]     order     Requested quadrature order.
    /// @param[in]     triangle  Vertices of the triangle to be clipped.
    /// @param[in]     box       Axis-aligned box to clip with.
    /// @param[out]    quadr     Output quadrature points and weights.
    /// @param[in,out] tmp       Optional quadrature object to use as temporary storage for internal
    ///                          per-triangle quadrature computation. Use this to efficiently
    ///                          compute integral over a set of boxes.
    ///
    static void clipped_triangle_box_quadrature(
        const int order,
        const TriangleVertices& triangle,
        const Eigen::AlignedBox2d& box,
        Quadrature& quadr,
        Quadrature* tmp = nullptr);

    ///
    /// Computes quadrature points and weights for a triangle clipped by a convex polygon.
    ///
    /// @note       This routine is not optimized and should only be used for testing purposes.
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
