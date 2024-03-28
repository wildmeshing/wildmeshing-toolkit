#pragma once


// #include <tbb/concurrent_vector.h>
// #include <tbb/enumerable_thread_specific.h>
// #include <tbb/parallel_for.h>

#include <array>
#include <iostream>
#include <wmtk/components/adaptive_tessellation/image/utils/sampling_utils.hpp>
#include <wmtk/utils/Logger.hpp>
#include "BarycentricTriangle.hpp"
#include "ThreeChannelPositionMapEvaluator.hpp"
#include "Triangle2DTo3DMapping.hpp"
namespace image = wmtk::components::image;
namespace wmtk::components::function::utils {

class MaxDistanceToLimit : public Triangle2DTo3DMapping
{
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


    Eigen::AlignedBox2d uv_triangle_bbox(
        const Eigen::Matrix<double, 3, 2, RowMajor>& uv_triangle_RowMajor) const;

    std::pair<int, double> pixel_num_size_of_uv_triangle(Eigen::AlignedBox2d& bbox) const;

    std::pair<int, double> pixel_size_of_uv_triangle(int pixel_num, Eigen::AlignedBox2d& bbox)
        const;
};
} // namespace wmtk::components::function::utils