
#pragma once
#include <Eigen/Dense>
#include <wmtk/Types.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/quadrature/PolygonClipping.hpp>
#include "BarycentricTriangle.hpp"

namespace wmtk::function::utils {
/**
 * @brief This function is used to find the intersections of the edge with the pixel grid. it return
 * a ordered list of intersection points in parametric form between point a and b
 *
 * @param m_three_channel_evaluator
 * @param a
 * @param b
 * @return std::vector<double>
 */
std::vector<double> pixel_grid_edge_intersections_in_barycentric(
    const wmtk::components::function::utils::ThreeChannelPositionMapEvaluator&
        m_three_channel_evaluator,
    const Vector2d& a,
    const Vector2d& b);
/**
 * @brief Given a parametric coordinate t that ranges between 0 and 1, this function finds the
 * point represented by t on the edge ab. It calculates the l2 norm of the distance from the PL
 * mesh to the limit surface at this point. Notice: ab needs to be arranged in the same order as
 * ab in function pixel_grid_edge_intersections_in_barycentric
 *
 * @param m_three_channel_evaluator
 * @param t
 * @param a
 * @param b
 * @return double the l2 distance to the limit surface at t on edge ab
 */
double l2_distance_to_limit_on_edge(
    const wmtk::components::function::utils::ThreeChannelPositionMapEvaluator&
        m_three_channel_evaluator,
    double t,
    const Vector2d& a,
    const Vector2d& b);

/**
 * @brief Find the maximum distance from any point on the edge ab to the limit surface. It returns
 * the maximum distance and the uv coordinate of the point on the edge that has the maximum
 * distance. Or if there is
 *
 * @param m_three_channel_evaluator
 * @param a
 * @param b
 * @return std::pair<double, Eigen::Vector2d>
 */
std::pair<double, Eigen::Vector2d> max_distance_and_uv_on_edge(
    const wmtk::components::function::utils::ThreeChannelPositionMapEvaluator&
        m_three_channel_evaluator,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b);

} // namespace wmtk::function::utils