#include "ClippedQuadrature.h"

#include <wmtk/utils/PolygonClipping.h>
#include "TriangleQuadrature.h"

namespace wmtk {

void ClippedQuadrature::clipped_triangle_box_quadrature(
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
{
    Eigen::MatrixXd current = polygon;
    Eigen::MatrixXd next;
    for (int i = 0; i < 3; ++i) {
        Eigen::RowVector2d q1 = triangle.row(i);
        Eigen::RowVector2d q2 = triangle.row((i + 1) % 3);
        clip_polygon_by_half_plane(current, q1, q2, next);
        std::swap(current, next);
    }
    Quadrature tmp;
    TriangleQuadrature rules;
    quadr.points.resize(0, 2);
    quadr.weights.resize(0);
    for (int i = 1; i + 1 < current.rows(); ++i) {
        TriangleVertices tri;
        tri.row(0) = current.row(0);
        tri.row(1) = current.row(i);
        tri.row(2) = current.row(i + 1);
        rules.transformed_triangle_quadrature(order, tri, tmp);
        quadr.points.conservativeResize(quadr.points.rows() + tmp.points.rows(), 2);
        quadr.weights.conservativeResize(quadr.weights.rows() + tmp.weights.rows());
        quadr.points.bottomRows(tmp.size()) = tmp.points;
        quadr.weights.tail(tmp.size()) = tmp.weights;
    }
}

} // namespace wmtk
