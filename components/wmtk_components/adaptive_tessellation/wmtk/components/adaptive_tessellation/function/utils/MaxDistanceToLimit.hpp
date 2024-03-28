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
};
} // namespace wmtk::components::function::utils