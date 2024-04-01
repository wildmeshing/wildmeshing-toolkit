#include "MaxDistanceToLimit.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <set>
#include <wmtk/components/adaptive_tessellation/quadrature/PolygonClipping.cpp>
#include <wmtk/components/adaptive_tessellation/quadrature/PolygonClipping.hpp>
#include <wmtk/utils/point_inside_triangle_check.hpp>
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
    // axis aligned half plane intersection each edge of the triangle
    // stores the intersection points of the half planes with every triangle edges (including
    // triangle vertices)
    std::vector<Eigen::RowVector2d> grid_line_intersections_01 =
        grid_line_intersections(uv0, uv1, bbox);
    std::vector<Eigen::RowVector2d> grid_line_intersections_12 =
        grid_line_intersections(uv1, uv2, bbox);
    std::vector<Eigen::RowVector2d> grid_line_intersections_20 =
        grid_line_intersections(uv2, uv0, bbox);


    // for all the grid points of the bbox, check if it's inside the triangle. If it is check their
    // value
    return value;
}

std::vector<Eigen::RowVector2d> MaxDistanceToLimit::grid_line_intersections(
    const Vector2d& a,
    const Vector2d& b,
    const Eigen::AlignedBox2d& bbox) const
{
    // for each horizontal and vertical halfplane of the bbox do line halfplane intersection
    auto compare = [](const Eigen::Vector2d& x, const Eigen::Vector2d& y) {
        // Replace this with some method of comparing Coordinates
        return x.x() != y.x() || x.y() != y.y();
    };

    std::set<Eigen::RowVector2d, decltype(compare)> intersections(compare);


    auto [xx1, yy1] = m_three_channel_evaluator.pixel_index(bbox.min());
    auto [xx2, yy2] = m_three_channel_evaluator.pixel_index(bbox.max());
    Eigen::Vector2d left, right, top, bottom;
    if (a.x() < b.x()) {
        left = a;
        right = b;
    } else {
        left = b;
        right = a;
    }
    if (a.y() < b.y()) {
        bottom = a;
        top = b;
    } else {
        bottom = b;
        top = a;
    }

    for (int i = 1; i <= xx2 - xx1; ++i) {
        // construct the halfplane with x = xxi
        AlignedHalfPlane<0, false> vertical{bbox.min().x() + (double)i};
        int status_left = point_is_in_aligned_half_plane(left, vertical);
        int status_right = point_is_in_aligned_half_plane(right, vertical);
        if (status_left > 0 && status_right < 0) {
            // the vertical line is between left and right
            // get the intersection
            Eigen::RowVector2d intersect;
            auto intersection = intersect_line_half_plane(left, right, vertical, intersect);
            if (intersection) {
                intersections.insert(intersect);
            }
        } else {
            break;
        }
    }
    for (int j = 1; j <= yy2 - yy1; ++j) {
        // construct the halfplane with x = xxi
        AlignedHalfPlane<1, false> horizontal{bbox.min().y() + (double)j};

        int status_bottom = point_is_in_aligned_half_plane(bottom, horizontal);
        int status_top = point_is_in_aligned_half_plane(top, horizontal);
        if (status_bottom > 0 && status_top < 0) {
            // the horizontal line is between left and right
            // get the intersection
            Eigen::RowVector2d intersect;
            auto intersection = intersect_line_half_plane(bottom, top, horizontal, intersect);
            if (intersection) {
                intersections.insert(intersect);
            }
        } else {
            break;
        }
    }
    intersections.insert(a);
    intersections.insert(b);
    // sort the intersections
    std::vector<Eigen::RowVector2d> sorted_intersections;
    for (const auto& inter : intersections) {
        sorted_intersections.push_back(inter);
    }
    std::sort(
        sorted_intersections.begin(),
        sorted_intersections.end(),
        [](const Eigen::RowVector2d& x, const Eigen::RowVector2d& y) { return x.x() < y.x(); });
    return sorted_intersections;
}
double MaxDistanceToLimit::l2_distance_to_limit(
    Eigen::Vector2d& uv,
    const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>& position_triangle_ColMajor,
    BarycentricTriangle<double>& bary) const
{
    Eigen::Vector3d texture_position = m_three_channel_evaluator.uv_to_position(uv);
    Eigen::Vector3d position = position_triangle_ColMajor * bary.get(uv);
    return (texture_position - position).norm();
}
double MaxDistanceToLimit::max_disatance_pixel_corners_inside_triangle(
    const Vector2d& uv0,
    const Vector2d& uv1,
    const Vector2d& uv2,
    const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>& position_triangle_ColMajor,
    const Eigen::AlignedBox2d& bbox) const
{
    double max_dis = 0.;
    BarycentricTriangle<double> bary(uv0, uv1, uv2);
    auto [num_pixels, pixel_size] = pixel_num_size_of_uv_triangle(bbox);

    auto check_four_corners = [&](Eigen::AlignedBox2d& box) -> double {
        double corner_max_dis = 0;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                Vector2d corner = box.min() + Vector2d(i, j) * pixel_size;
                if (wmtk::utils::point_inside_triangle_2d_check(uv0, uv1, uv2, corner)) {
                    double dis = l2_distance_to_limit(corner, position_triangle_ColMajor, bary);
                    corner_max_dis = std::max(corner_max_dis, dis);
                }
            }
        }
        return corner_max_dis;
    };
    for (auto y = 0; y < num_pixels; ++y) {
        for (auto x = 0; x < num_pixels; ++x) {
            Eigen::AlignedBox2d box;
            box.extend(bbox.min() + Eigen::Vector2d(x * pixel_size, y * pixel_size));
            box.extend(bbox.min() + Eigen::Vector2d((x + 1) * pixel_size, (y + 1) * pixel_size));
            // check the four corners of the pixel, if any of them is inside the triangle.
            // if it is, calculate the distance
            max_dis = std::max(max_dis, check_four_corners(box));
        }
    }
    return max_dis;
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
    const Eigen::AlignedBox2d& bbox) const
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
    const Eigen::AlignedBox2d& bbox) const
{
    assert(pixel_num > 0);
    double pixel_size = bbox.diagonal().maxCoeff() / pixel_num;
    return {pixel_num, pixel_size};
}
} // namespace wmtk::components::function::utils