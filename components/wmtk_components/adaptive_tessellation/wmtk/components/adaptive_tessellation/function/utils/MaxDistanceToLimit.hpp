#pragma once


// #include <tbb/concurrent_vector.h>
// #include <tbb/enumerable_thread_specific.h>
// #include <tbb/parallel_for.h>

#include <array>
#include <iostream>
#include <set>
#include <wmtk/components/adaptive_tessellation/image/utils/sampling_utils.hpp>
#include <wmtk/utils/Logger.hpp>
#include "BarycentricTriangle.hpp"
#include "ThreeChannelPositionMapEvaluator.hpp"
#include "Triangle2DTo3DMapping.hpp"
#include "pixel_image_triangle_helper.hpp"
namespace image = wmtk::components::image;
namespace wmtk::components::function::utils {

class MaxDistanceToLimit : public Triangle2DTo3DMapping
{
    class GridLineIntersections
    {
    public:
        std::vector<Eigen::RowVector2d> intersections;
    };

public:
    MaxDistanceToLimit(const ThreeChannelPositionMapEvaluator& evaluator);


public:
    // given a triangle represented by three 2d vertices uv0, uv2, uv3. We wanto find the maximum
    // distance from any point in the triangle, mapped to the 3d embedding space, to the limit
    // surface in the same embedding space the piecewise linear mesh is approximating
    double distance(
        const Vector2<double>& uv0,
        const Vector2<double>& uv1,
        const Vector2<double>& uv2) const override;
    DScalar distance(
        const Vector2<DScalar>& uv0,
        const Vector2<DScalar>& uv1,
        const Vector2<DScalar>& uv2) const override
    {
        throw std::runtime_error("DScalar type in MaxDistanceToLimit Not implemented");
    }

    std::vector<double> _debug_grid_line_intersections(const Vector2d& a, const Vector2d& b) const
    {
        return wmtk::function::utils::pixel_grid_edge_intersections_in_barycentric(
            m_three_channel_evaluator,
            a,
            b);
    }

    double _debug_max_disatance_pixel_corners_inside_triangle(
        const Vector2d& uv0,
        const Vector2d& uv1,
        const Vector2d& uv2,
        const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>& position_triangle_ColMajor,
        const Eigen::AlignedBox2d& bbox) const
    {
        return max_disatance_pixel_corners_inside_triangle(
            uv0,
            uv1,
            uv2,
            position_triangle_ColMajor,
            bbox);
    }

protected:
    template <typename T>
    Eigen::AlignedBox2d
    uv_triangle_bbox(const Vector2<T>& uv0, const Vector2<T>& uv1, const Vector2<T>& uv2) const
    {
        Eigen::AlignedBox2d bbox;
        bbox.extend(Vector2d(image::utils::get_double(uv0)));
        bbox.extend(Vector2d(image::utils::get_double(uv1)));
        bbox.extend(Vector2d(image::utils::get_double(uv2)));
        return bbox;
    }

    double pixel_coord_l2_distance_to_limit(
        Eigen::Vector2d& pixel_uv,
        const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>& position_triangle_ColMajor,
        BarycentricTriangle<double>& bary) const;

    double max_distance_on_line_segment(
        const Vector2d& a,
        const Vector2d& b,
        const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>& position_triangle_ColMajor,
        const BarycentricTriangle<double>& bary) const;

    double max_disatance_pixel_corners_inside_triangle(
        const Vector2d& uv0,
        const Vector2d& uv1,
        const Vector2d& uv2,
        const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>& position_triangle_ColMajor,
        const Eigen::AlignedBox2d& bbox) const;
};
} // namespace wmtk::components::function::utils