#include "AnalyticalFunctionNumericalIntegral.hpp"

#include <wmtk/components/adaptive_tessellation/quadrature/ClippedQuadrature.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "BarycentricTriangle.hpp"

using namespace Eigen;
namespace wmtk::components {
namespace function::utils {

AnalyticalFunctionNumericalIntegral::AnalyticalFunctionNumericalIntegral(
    const ThreeChannelPositionMapEvaluator& evaluator)
    : m_three_channel_evaluator(evaluator)
{
    std::cout << "==== AnalyticalFunctionNumericalIntegral constructor" << std::endl;
}

AnalyticalFunctionNumericalIntegral::~AnalyticalFunctionNumericalIntegral() = default;

double AnalyticalFunctionNumericalIntegral::triangle_quadrature(
    const Vector2<double>& uv0,
    const Vector2<double>& uv1,
    const Vector2<double>& uv2) const
{
    return triangle_quadrature_T<double>(uv0, uv1, uv2);
}
DScalar AnalyticalFunctionNumericalIntegral::triangle_quadrature(
    const Vector2<DScalar>& uv0,
    const Vector2<DScalar>& uv1,
    const Vector2<DScalar>& uv2) const
{
    return triangle_quadrature_T<DScalar>(uv0, uv1, uv2);
}
} // namespace function::utils
} // namespace wmtk::components