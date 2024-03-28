#pragma once
#include <wmtk/function/utils/autodiff.h>
#include <wmtk/components/adaptive_tessellation/image/utils/SamplingParameters.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "BarycentricTriangle.hpp"
#include "ThreeChannelPositionMapEvaluator.hpp"

namespace image = wmtk::components::image;
namespace wmtk::components::function::utils {
using DScalar = typename wmtk::function::PerSimplexAutodiffFunction::DScalar;
class Triangle2DTo3DMapping
{
public:
    // enum class IntegrationMethod { Exact, Adaptive };

    virtual ~Triangle2DTo3DMapping() = default;
    Triangle2DTo3DMapping(const ThreeChannelPositionMapEvaluator& evaluator)
        : m_three_channel_evaluator(evaluator)
    {}

    virtual double distance(
        const Vector2<double>& uv0,
        const Vector2<double>& uv1,
        const Vector2<double>& uv2) const
    {
        throw std::runtime_error("Not implemented");
    };
    virtual DScalar distance(
        const Vector2<DScalar>& uv0,
        const Vector2<DScalar>& uv1,
        const Vector2<DScalar>& uv2) const
    {
        throw std::runtime_error("Not implemented");
    };

protected:
    const ThreeChannelPositionMapEvaluator& m_three_channel_evaluator;
};

} // namespace wmtk::components::function::utils
