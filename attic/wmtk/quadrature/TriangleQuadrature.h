#pragma once

#include "Quadrature.h"

namespace wmtk {

class TriangleQuadrature
{
public:
    using TriangleVertices = Eigen::Matrix<double, 3, 2, Eigen::RowMajor>;

public:
    ///
    /// Computes quadrature points and weights for an arbitrary 2D triangle.
    ///
    /// @param[in]  order     Requested quadrature order.
    /// @param[in]  triangle  Vertices of the transformed 2D triangle.
    /// @param[out] quadr     Output quadrature points and weights.
    ///
    static void transformed_triangle_quadrature(
        const int order,
        const TriangleVertices& triangle,
        Quadrature& quadr);

    ///
    /// Computes quadrature points and weights for the reference 2D triangle (0,0), (1,0), (0,1).
    ///
    /// @param[in]  order  Requested quadrature order.
    /// @param[out] quadr  Output quadrature points and weights.
    ///
    static void reference_triangle_quadrature(const int order, Quadrature& quadr);
};

} // namespace wmtk
