#include "pixel_image_triangle_helper.hpp"
#include <Eigen/Geometry>
#include <set>
#include <wmtk/utils/triangle_areas.hpp>
namespace wmtk::function::utils {

std::vector<double> pixel_grid_edge_intersections_in_barycentric(
    const wmtk::components::function::utils::ThreeChannelPositionMapEvaluator&
        m_three_channel_evaluator,
    const Vector2d& a,
    const Vector2d& b)
{
    std::vector<double> intersections;

    Eigen::AlignedBox2d bbox;
    bbox.extend(a);
    bbox.extend(b);
    auto [xx1, yy1] = m_three_channel_evaluator.pixel_index_floor(bbox.min());
    auto [xx2, yy2] = m_three_channel_evaluator.pixel_index_ceil(bbox.max());
    int size = std::max(m_three_channel_evaluator.width(), m_three_channel_evaluator.height());
    Eigen::Vector2d left, right, top, bottom;
    // covert a and b to pixel coordinate i.e. ranging from 0 to image.size()
    Eigen::Vector2d pixel_a = a * size;
    Eigen::Vector2d pixel_b = b * size;
    if (a.x() < b.x()) {
        left = pixel_a;
        right = pixel_b;
    } else {
        left = pixel_b;
        right = pixel_a;
    }
    if (a.y() < b.y()) {
        bottom = pixel_a;
        top = pixel_b;
    } else {
        bottom = pixel_b;
        top = pixel_a;
    }
    double pixel_size = m_three_channel_evaluator.pixel_size();

    for (int i = 0; i <= xx2 - xx1; ++i) {
        // construct the halfplane with x = xxi
        AlignedHalfPlane<0, true> vertical{0.5 + i + xx1};

        int status_floor = point_is_in_aligned_half_plane(left, vertical);
        int status_ceil = point_is_in_aligned_half_plane(right, vertical);

        if (status_floor > 0 && status_ceil < 0) {
            // the plane line is between floor and ceil
            // get the intersection
            double t = intersect_line_half_plane_in_barycentric(pixel_a, pixel_b, vertical);
            if (t >= 0) {
                intersections.emplace_back(t);
            }
        } else {
            continue;
        }
    }
    for (int j = 0; j <= yy2 - yy1; ++j) {
        // construct the halfplane with y = yyi
        AlignedHalfPlane<1, true> horizontal{0.5 + j + yy1};
        int status_floor = point_is_in_aligned_half_plane(bottom, horizontal);
        int status_ceil = point_is_in_aligned_half_plane(top, horizontal);

        if (status_floor > 0 && status_ceil < 0) {
            // the plane line is between floor and ceil
            // get the intersection
            double t = intersect_line_half_plane_in_barycentric(pixel_a, pixel_b, horizontal);
            if (t >= 0) {
                intersections.emplace_back(t);
            }
        } else {
            continue;
        }
    }
    // insert the two end points
    intersections.emplace_back(0.);
    intersections.emplace_back(1.);
    std::sort(intersections.begin(), intersections.end());
    return intersections;
}

double l2_distance_to_limit_on_edge(
    const wmtk::components::function::utils::ThreeChannelPositionMapEvaluator&
        m_three_channel_evaluator,
    double t,
    const Vector2d& a,
    const Vector2d& b)
{
    Eigen::Vector3d pa = m_three_channel_evaluator.uv_to_position(a);
    Eigen::Vector3d pb = m_three_channel_evaluator.uv_to_position(b);
    Eigen::Vector2d t_uv = a * (1 - t) + b * t;

    Eigen::Vector3d limit_position = m_three_channel_evaluator.uv_to_position(t_uv);
    Eigen::Vector3d position = pa * (1 - t) + pb * t;
    return (limit_position - position).norm();
}

std::pair<double, Eigen::Vector2d> max_distance_and_uv_on_edge(
    const wmtk::components::function::utils::ThreeChannelPositionMapEvaluator&
        m_three_channel_evaluator,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b)
{
    double max_dist = 0;
    Eigen::Vector2d max_uv = (a + b) / 2.;
    auto intersections =
        pixel_grid_edge_intersections_in_barycentric(m_three_channel_evaluator, a, b);
    for (double t : intersections) {
        double dist = l2_distance_to_limit_on_edge(m_three_channel_evaluator, t, a, b);
        if (dist > max_dist) {
            max_dist = dist;
            if (t == 0. || t == 1.) continue;
            max_uv = a * (1 - t) + b * t;
        }
    }
    return {max_dist, max_uv};
}

Eigen::Vector2d get_best_uv(
    const std::shared_ptr<wmtk::components::function::utils::Triangle2DTo3DMapping>& m_mapping_ptr,
    const Eigen::Vector2d& edge_uv0,
    const Eigen::Vector2d& edge_uv1,
    const Eigen::Vector2d& top_uv,
    const std::optional<Eigen::Vector2d>& btm_uv_opt)
{
    int sample_size = 8;
    double t_size = 1.0 / (sample_size + 1);

    Eigen::Vector2d best_uv = (edge_uv0 + edge_uv1) / 2;
    double mini_distance = m_mapping_ptr->distance(best_uv, top_uv, edge_uv0) +
                           m_mapping_ptr->distance(best_uv, top_uv, edge_uv1);
    if (btm_uv_opt) {
        mini_distance += m_mapping_ptr->distance(best_uv, btm_uv_opt.value(), edge_uv0) +
                         m_mapping_ptr->distance(best_uv, btm_uv_opt.value(), edge_uv1);
    }
    for (int i = 0; i < sample_size; i++) {
        double t = (i + 1) * t_size;
        Eigen::Vector2d new_uv = (1 - t) * edge_uv0 + t * edge_uv1;

        double new_distance = m_mapping_ptr->distance(new_uv, top_uv, edge_uv0) +
                              m_mapping_ptr->distance(new_uv, top_uv, edge_uv1);
        double avg_tri_area =
            wmtk::utils::triangle_unsigned_2d_area(edge_uv0, edge_uv1, top_uv) / 2;

        if (btm_uv_opt) {
            new_distance += m_mapping_ptr->distance(new_uv, btm_uv_opt.value(), edge_uv0) +
                            m_mapping_ptr->distance(new_uv, btm_uv_opt.value(), edge_uv1);
            double other_tri_area =
                wmtk::utils::triangle_unsigned_2d_area(edge_uv0, edge_uv1, btm_uv_opt.value()) / 2;
            avg_tri_area = (avg_tri_area + other_tri_area) / 2;
        }


        if (abs(new_distance - mini_distance) < 1e-6 * avg_tri_area) {
            // logger().info("new_distance == mini_distance");
            continue;
        }
        if (new_distance < mini_distance) {
            mini_distance = new_distance;
            best_uv = new_uv;
        }
    }
    return best_uv;
}
} // namespace wmtk::function::utils