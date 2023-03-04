#include "ClippedQuadrature.h"

namespace wmtk {

void ClippedQuadrature::clipped_triangle_quad_quadrature(
    const int order,
    const TriangleVertices& triangle,
    const Eigen::AlignedBox2d& box,
    Quadrature& quadr)
{}

void ClippedQuadrature::clipped_triangle_polygon_quadrature(
    const int order,
    const TriangleVertices& triangle,
    const PolygonVertices& polygon,
    Quadrature& quadr)
{}

} // namespace wmtk
