#include "ClippedQuadrature.hpp"

#include "PolygonClipping.hpp"
#include "TriangleQuadrature.hpp"

namespace wmtk {

void ClippedQuadrature::clipped_triangle_box_quadrature(
    const int order,
    const TriangleVertices& triangle,
    const Eigen::AlignedBox2d& box,
    Quadrature& quadr,
    Quadrature* tmp)
{
#if 0
    // Naive implementation for testing
    PolygonVertices poly(4, 2);
    poly.row(0) = box.corner(Eigen::AlignedBox2d::CornerType::BottomLeft).transpose();
    poly.row(1) = box.corner(Eigen::AlignedBox2d::CornerType::BottomRight).transpose();
    poly.row(2) = box.corner(Eigen::AlignedBox2d::CornerType::TopRight).transpose();
    poly.row(3) = box.corner(Eigen::AlignedBox2d::CornerType::TopLeft).transpose();
    clipped_triangle_polygon_quadrature(order, triangle, poly, quadr);
#else
    auto clipped = clip_triangle_by_box(triangle, box);

    Quadrature dummy;
    if (tmp == nullptr) {
        tmp = &dummy;
    }

    TriangleQuadrature rules;
    quadr.set_dimension(2);
    quadr.clear();
    for (int i = 1; i + 1 < clipped.rows(); ++i) {
        TriangleVertices tri;
        tri.row(0) = clipped.row(0);
        tri.row(1) = clipped.row(i);
        tri.row(2) = clipped.row(i + 1);
        rules.transformed_triangle_quadrature(order, tri, *tmp);
        quadr.resize(quadr.size() + tmp->size());
        quadr.points().bottomRows(tmp->size()) = tmp->points();
        quadr.weights().tail(tmp->size()) = tmp->weights();
    }
#endif
}

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
    quadr.set_dimension(2);
    quadr.clear();
    for (int i = 1; i + 1 < current.rows(); ++i) {
        TriangleVertices tri;
        tri.row(0) = current.row(0);
        tri.row(1) = current.row(i);
        tri.row(2) = current.row(i + 1);
        rules.transformed_triangle_quadrature(order, tri, tmp);
        quadr.resize(quadr.size() + tmp.size());
        quadr.points().bottomRows(tmp.size()) = tmp.points();
        quadr.weights().tail(tmp.size()) = tmp.weights();
    }
}

} // namespace wmtk
