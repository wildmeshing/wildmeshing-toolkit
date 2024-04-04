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

    Eigen::AlignedBox2d bbox = uv_triangle_bbox(uv_triangle_RowMajor);


    // for all the grid points of the bbox, check if it's inside the triangle. If it is check their
    // value
    double max_dist = max_disatance_pixel_corners_inside_triangle(
        uv0,
        uv1,
        uv2,
        position_triangle_ColMajor,
        bbox);

    // axis aligned half plane intersection each edge of the triangle
    // stores the intersection points of the half planes with every triangle edges (including
    // triangle vertices)
    std::vector<Eigen::RowVector2d> grid_line_intersections_01 =
        grid_line_intersections(uv0, uv1, bbox);
    std::vector<Eigen::RowVector2d> grid_line_intersections_12 =
        grid_line_intersections(uv1, uv2, bbox);
    std::vector<Eigen::RowVector2d> grid_line_intersections_20 =
        grid_line_intersections(uv2, uv0, bbox);
    // check all the intersections collected
    for (int64_t i = 0; i < grid_line_intersections_01.size() - 1; ++i) {
        Eigen::Vector2d inter = grid_line_intersections_01[i];
        Eigen::Vector2d next_inter = grid_line_intersections_01[i + 1];
        max_dist =
            std::max(max_dist, l2_distance_to_limit(inter, position_triangle_ColMajor, bary));
        max_dist = std::max(
            max_dist,
            max_distance_on_line_segment(inter, next_inter, position_triangle_ColMajor, bary));
    }
    for (int64_t i = 0; i < grid_line_intersections_12.size() - 1; ++i) {
        Eigen::Vector2d inter = grid_line_intersections_12[i];
        Eigen::Vector2d next_inter = grid_line_intersections_12[i + 1];
        max_dist =
            std::max(max_dist, l2_distance_to_limit(inter, position_triangle_ColMajor, bary));
        max_dist = std::max(
            max_dist,
            max_distance_on_line_segment(inter, next_inter, position_triangle_ColMajor, bary));
    }
    for (int64_t i = 0; i < grid_line_intersections_20.size() - 1; ++i) {
        Eigen::Vector2d inter = grid_line_intersections_20[i];
        Eigen::Vector2d next_inter = grid_line_intersections_20[i + 1];
        max_dist =
            std::max(max_dist, l2_distance_to_limit(inter, position_triangle_ColMajor, bary));
        max_dist = std::max(
            max_dist,
            max_distance_on_line_segment(inter, next_inter, position_triangle_ColMajor, bary));
    }

    return max_dist;
}

// include two end points of the edge in the ordered intersection list
// the vector is order by x then y of each intersection point
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


    auto [xx1, yy1] = m_three_channel_evaluator.pixel_index_floor(bbox.min());
    auto [xx2, yy2] = m_three_channel_evaluator.pixel_index_ceil(bbox.max());
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
        int status_floor = point_is_in_aligned_half_plane(left, vertical);
        int status_ceil = point_is_in_aligned_half_plane(right, vertical);
        if (status_floor > 0 && status_ceil < 0) {
            // the plane line is between floor and ceil
            // get the intersection
            Eigen::RowVector2d intersect;
            auto intersection = intersect_line_half_plane(left, right, vertical, intersect);
            if (intersection) {
                intersections.insert(intersect);
            }
        } else {
            continue;
        }
    }
    for (int j = 1; j <= yy2 - yy1; ++j) {
        // construct the halfplane with y = yyi
        AlignedHalfPlane<1, false> horizontal{bbox.min().y() + (double)j};
        int status_floor = point_is_in_aligned_half_plane(bottom, horizontal);
        int status_ceil = point_is_in_aligned_half_plane(top, horizontal);
        if (status_floor > 0 && status_ceil < 0) {
            // the plane line is between floor and ceil
            // get the intersection
            Eigen::RowVector2d intersect;
            auto intersection = intersect_line_half_plane(bottom, top, horizontal, intersect);
            if (intersection) {
                intersections.insert(intersect);
            }
        } else {
            continue;
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
    auto [xx1, yy1] = m_three_channel_evaluator.pixel_index_floor(bbox.min());
    auto [xx2, yy2] = m_three_channel_evaluator.pixel_index_ceil(bbox.max());

    auto check_four_corners = [&](int xmin, int ymin) -> double {
        double corner_max_dis = 0;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                auto u = (xmin + i) * m_three_channel_evaluator.pixel_size();
                auto v = (ymin + j) * m_three_channel_evaluator.pixel_size();
                Eigen::Vector2d uv = {u, v};
                if (wmtk::utils::point_inside_triangle_2d_check(uv0, uv1, uv2, uv)) {
                    corner_max_dis = std::max(
                        corner_max_dis,
                        l2_distance_to_limit(uv, position_triangle_ColMajor, bary));
                }
            }
        }
        return corner_max_dis;
    };
    for (auto y = yy1; y < yy2; ++y) {
        for (auto x = xx1; x < xx2; ++x) {
            // check the four corners of the pixel, if any of them is inside the triangle.
            // if it is, calculate the distance
            max_dis = std::max(max_dis, check_four_corners(xx1, yy1));
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

double MaxDistanceToLimit::max_distance_on_line_segment(
    const Vector2d& a,
    const Vector2d& b,
    const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>& position_triangle_ColMajor,
    const BarycentricTriangle<double>& bary) const
{
    double max_dist = 0;
    // assume a is left, b is right
    Eigen::AlignedBox2d line_bbox;
    line_bbox.extend(a);
    line_bbox.extend(b);
    // get the pixel coordinate for the 4 corners where a and b are in
    auto [xx1, yy1] = m_three_channel_evaluator.pixel_index_floor(line_bbox.min());
    auto [xx2, yy2] = m_three_channel_evaluator.pixel_index_ceil(line_bbox.max());
    // this should be in the same pixel
    assert(((xx2 - xx1 < 1) && (yy2 - yy1 < 1)));

    // get the pixel corner value
    // left, right, bottom, top
    Eigen::Vector3d lb, rb, lt, rt;
    // lb = m_three_channel_evaluator.pixel_index_to_position<double>(xx1, yy1);
    // rb = m_three_channel_evaluator.pixel_index_to_position<double>(xx2, yy1);
    // lt = m_three_channel_evaluator.pixel_index_to_position<double>(xx1, yy2);
    // rt = m_three_channel_evaluator.pixel_index_to_position<double>(xx2, yy2);

    // // check if the the max value is taken between the two points
    // // solution to the constraint extrema (derived by sympy)
    // Eigen::Vector2d p_max = {
    //     xx2 * m_three_channel_evaluator.pixel_size(),
    //     yy2 * m_three_channel_evaluator.pixel_size()};
    // Eigen::Vector2d p_min = {
    //     xx1 * m_three_channel_evaluator.pixel_size(),
    //     yy1 * m_three_channel_evaluator.pixel_size()};

    // Eigen::Vector3d top = -lb * a.x() * b.y() + lb * a.x() * p_max.y() + lb * a.y() * b.x() +
    //                       lb * a.y() * p_max.x() - lb * b.x() * p_max.y() - lb * b.y() *
    //                       p_max.x() + rb * a.x() * b.y() - rb * a.x() * p_max.y() - rb * a.y() *
    //                       b.x() - rb * a.y() * p_min.x() + rb * b.x() * p_max.y() + rb * b.y() *
    //                       p_min.x() + lt * a.x() * b.y() - lt * a.x() * p_min.y() - lt * a.y() *
    //                       b.x() - lt * a.y() * p_max.x() + lt * b.x() * p_min.y() + lt * b.y() *
    //                       p_max.x() - rt * a.x() * b.y() + rt * a.x() * p_min.y() + rt * a.y() *
    //                       b.x() + rt * a.y() * p_min.x() - rt * b.x() * p_min.y() - rt * b.y() *
    //                       p_min.x();
    // Eigen::Vector3d bot = 2 * (lb * a.y() - lb * b.y() - rb * a.y() + rb * b.y() - lt * a.y() +
    //                            lt * b.y() + rt * a.y() - rt * b.y());
    // double u = (top).cwiseQuotient(bot);
    // double v = a.y() + (b.y() - a.y()) * (u - a.x()) / (b.x() - a.x());
    // if (u > line_bbox.max().x() || u < line_bbox.min().x() || v > line_bbox.max().y() ||
    //     v < line_bbox.min().y()) {
    //     return max_dist;
    // }
    // max_dist = std::max(max_dist, l2_distance_to_limit({u, v}, position_triangle_ColMajor,
    // bary));

    return max_dist;
}

} // namespace wmtk::components::function::utils