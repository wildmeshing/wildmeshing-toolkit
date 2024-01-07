#include "AnalyticalFunctionTriangleQuadrature.hpp"

#include <wmtk/components/adaptive_tessellation/quadrature/ClippedQuadrature.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "BarycentricTriangle.hpp"

using namespace Eigen;
namespace wmtk::components::adaptive_tessellation {
namespace function::utils {

AnalyticalFunctionTriangleQuadrature::AnalyticalFunctionTriangleQuadrature(
    const ThreeChannelPositionMapEvaluator& evaluator)
    : m_three_channel_evaluator(evaluator)
{}

AnalyticalFunctionTriangleQuadrature::~AnalyticalFunctionTriangleQuadrature() = default;
} // namespace function::utils
} // namespace wmtk::components::adaptive_tessellation