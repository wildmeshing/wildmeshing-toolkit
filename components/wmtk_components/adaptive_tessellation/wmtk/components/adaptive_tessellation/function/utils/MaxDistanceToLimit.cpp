#include "MaxDistanceToLimit.hpp"
#include <wmtk/components/adaptive_tessellation/quadrature/PolygonClipping.hpp>
namespace wmtk::components::function::utils {
MaxDistanceToLimit::MaxDistanceToLimit(const ThreeChannelPositionMapEvaluator& evaluator)
    : Triangle2DTo3DMapping(evaluator)
{}

double MaxDistanceToLimit::distance(
    const Vector2<double>& uv0,
    const Vector2<double>& uv1,
    const Vector2<double>& uv2) const
{
    Vector3<double> p0, p1, p2;
    p0 = m_three_channel_evaluator.uv_to_position(uv0);
    p1 = m_three_channel_evaluator.uv_to_position(uv1);
    p2 = m_three_channel_evaluator.uv_to_position(uv2);

    Eigen::Matrix<double, 3, 3, Eigen::ColMajor> position_triangle_ColMajor;
    position_triangle_ColMajor.col(0) = p0;
    position_triangle_ColMajor.col(1) = p1;
    position_triangle_ColMajor.col(2) = p2;

    Eigen::Matrix<double, 3, 2, RowMajor> uv_triangle_RowMajor;
    uv_triangle_RowMajor.row(0) = image::utils::get_double(uv0);
    uv_triangle_RowMajor.row(1) = image::utils::get_double(uv1);
    uv_triangle_RowMajor.row(2) = image::utils::get_double(uv2);

    // std::cout << "uv0 " << uv_triangle_RowMajor.row(0) << std::endl;
    // std::cout << "uv1 " << uv_triangle_RowMajor.row(1) << std::endl;
    // std::cout << "uv2 " << uv_triangle_RowMajor.row(2) << std::endl;

    // calculate the barycentric coordinate of the a point using u, v cooridnates
    // returns the 3d coordinate on the current mesh
    BarycentricTriangle<double> bary(uv0, uv1, uv2);
    if (bary.is_degenerate()) {
        return 0.;
    }

    double value = 0.;
    Eigen::AlignedBox2d bbox = uv_triangle_bbox(uv_triangle_RowMajor);
    auto [num_pixels, pixel_size] = pixel_num_size_of_uv_triangle(bbox);

    for (auto y = 0; y < num_pixels; ++y) {
        for (auto x = 0; x < num_pixels; ++x) {
            Eigen::AlignedBox2d box;
            box.extend(bbox.min() + Eigen::Vector2d(x * pixel_size, y * pixel_size));
            box.extend(bbox.min() + Eigen::Vector2d((x + 1) * pixel_size, (y + 1) * pixel_size));
            auto clipped = clip_triangle_by_box(uv_triangle_RowMajor, box);
            double max_distance = 0.;
            for (int i = 0; i < clipped.rows(); ++i) {
                Vector2d intersection_uv = clipped.row(i);
                Vector3d texture_position =
                    m_three_channel_evaluator.uv_to_position(intersection_uv);
                Vector3d position = position_triangle_ColMajor * bary.get(intersection_uv);
                Vector3d diffp = texture_position - position;
                if (max_distance < diffp.norm()) {
                    max_distance = diffp.norm();
                }
            }
            // now check the four corners of the pixels that are in the triangle
            //
        }
    }
    // now check the three vertices of the triangle
    return value;
}

Eigen::AlignedBox2d MaxDistanceToLimit::uv_triangle_bbox(
    const Eigen::Matrix<double, 3, 2, RowMajor>& uv_triangle_RowMajor) const
{
    Eigen::AlignedBox2d bbox;
    bbox.extend(uv_triangle_RowMajor.row(0).transpose());
    bbox.extend(uv_triangle_RowMajor.row(1).transpose());
    bbox.extend(uv_triangle_RowMajor.row(2).transpose());
    return bbox;
}
std::pair<int, double> MaxDistanceToLimit::pixel_num_size_of_uv_triangle(
    Eigen::AlignedBox2d& bbox) const
{
    Vector2d bbox_min = bbox.min();
    Vector2d bbox_max = bbox.max();
    auto [xx1, yy1] = m_three_channel_evaluator.pixel_index(bbox_min);
    auto [xx2, yy2] = m_three_channel_evaluator.pixel_index(bbox_max);

    int num_pixels = std::max(abs(xx2 - xx1), abs(yy2 - yy1)) + 1;
    assert(num_pixels > 0);
    double pixel_size = bbox.diagonal().maxCoeff() / num_pixels;
    return {num_pixels, pixel_size};
}

std::pair<int, double> MaxDistanceToLimit::pixel_size_of_uv_triangle(
    int pixel_num,
    Eigen::AlignedBox2d& bbox) const
{
    assert(pixel_num > 0);
    double pixel_size = bbox.diagonal().maxCoeff() / pixel_num;
    return {pixel_num, pixel_size};
}
} // namespace wmtk::components::function::utils