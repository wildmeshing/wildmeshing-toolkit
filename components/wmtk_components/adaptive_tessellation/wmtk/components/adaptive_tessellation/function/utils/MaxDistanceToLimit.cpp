#include "MaxDistanceToLimit.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
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


    // calculate the barycentric coordinate of the a point using u, v cooridnates in the pixel grid
    // So the coordinate ranges from [0, image.size()] instead of [0, 1]
    auto size = m_three_channel_evaluator.image_size();
    BarycentricTriangle<double> bary(uv0 * size, uv1 * size, uv2 * size);
    if (bary.is_degenerate()) {
        return 0.;
    }

    Eigen::AlignedBox2d bbox;
    bbox.extend(uv0);
    bbox.extend(uv1);
    bbox.extend(uv2);


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
    for (int64_t i = 0; i < grid_line_intersections_01.size(); ++i) {
        Eigen::Vector2d inter = grid_line_intersections_01[i];

        auto tmp = pixel_coord_l2_distance_to_limit(inter, position_triangle_ColMajor, bary);
        if (tmp > max_dist) {
            max_dist = std::max(max_dist, tmp);
        }
    }
    for (int64_t i = 0; i < grid_line_intersections_12.size(); ++i) {
        Eigen::Vector2d inter = grid_line_intersections_12[i];
        auto tmp = pixel_coord_l2_distance_to_limit(inter, position_triangle_ColMajor, bary);
        if (tmp > max_dist) {
            max_dist = std::max(max_dist, tmp);
        }
    }
    for (int64_t i = 0; i < grid_line_intersections_20.size(); ++i) {
        Eigen::Vector2d inter = grid_line_intersections_20[i];
        auto tmp = pixel_coord_l2_distance_to_limit(inter, position_triangle_ColMajor, bary);
        if (tmp > max_dist) {
            max_dist = std::max(max_dist, tmp);
        }
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
    int size = std::max(m_three_channel_evaluator.width(), m_three_channel_evaluator.height());
    Eigen::Vector2d left, right, top, bottom;

    if (a.x() < b.x()) {
        left = a * size;
        right = b * size;
    } else {
        left = b * size;
        right = a * size;
    }
    if (a.y() < b.y()) {
        bottom = a * size;
        top = b * size;
    } else {
        bottom = b * size;
        top = a * size;
    }
    double pixel_size = m_three_channel_evaluator.pixel_size();

    for (int i = 0; i <= xx2 - xx1; ++i) {
        // construct the halfplane with x = xxi
        AlignedHalfPlane<0, true> vertical{0.5 + i};

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
    for (int j = 0; j <= yy2 - yy1; ++j) {
        // construct the halfplane with y = yyi
        AlignedHalfPlane<1, true> horizontal{0.5 + j};
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
    intersections.insert(a * size);
    intersections.insert(b * size);
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

/**
 * @brief the uv input for this function is from grid_line_intersections, where the uv are in the
 * pixel grid coordinate. uv is in [0, image.size()] instead of [0, 1]. But position coordinate and
 * bary are unchanged
 *
 * @param uv
 * @param position_triangle_ColMajor
 * @param bary
 * @return double
 */
double MaxDistanceToLimit::pixel_coord_l2_distance_to_limit(
    Eigen::Vector2d& pixel_uv,
    const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>& position_triangle_ColMajor,
    BarycentricTriangle<double>& bary) const
{
    Eigen::Vector3d texture_position =
        m_three_channel_evaluator.pixel_coord_to_position_bilinear(pixel_uv);
    Eigen::Vector3d position = position_triangle_ColMajor * bary.get(pixel_uv);
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
    auto size = m_three_channel_evaluator.image_size();
    auto pixel_uv0 = uv0 * size;
    auto pixel_uv1 = uv1 * size;
    auto pixel_uv2 = uv2 * size;

    BarycentricTriangle<double> bary(pixel_uv0, pixel_uv1, pixel_uv2);
    auto [xx1, yy1] = m_three_channel_evaluator.pixel_index_floor(bbox.min());
    auto [xx2, yy2] = m_three_channel_evaluator.pixel_index_ceil(bbox.max());
    auto check_center = [&](int xmin, int ymin) -> double {
        double _max_dist = 0;
        auto pixel_u = xmin + 0.5;
        auto pixel_v = ymin + 0.5;
        Eigen::Vector2d pixel_uv = {pixel_u, pixel_v};
        if (wmtk::utils::point_inside_triangle_2d_check(
                pixel_uv0,
                pixel_uv1,
                pixel_uv2,
                pixel_uv)) {
            auto temp =
                pixel_coord_l2_distance_to_limit(pixel_uv, position_triangle_ColMajor, bary);

            if (temp > _max_dist) {
                _max_dist = temp;
            }
        }
        return _max_dist;
    };
    for (auto y = yy1; y <= yy2; ++y) {
        for (auto x = xx1; x <= xx2; ++x) {
            // check the center of the pixel, if any of them is inside the triangle.
            // if it is, calculate the distance
            max_dis = std::max(max_dis, check_center(x, y));
        }
    }
    return max_dis;
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