#include "AnalyticalFunctionTriangleQuadrature.hpp"

#include <wmtk/components/adaptive_tessellation/quadrature/ClippedQuadrature.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "BarycentricTriangle.hpp"

using namespace Eigen;
namespace wmtk::components {
namespace function::utils {

AnalyticalFunctionTriangleQuadrature::AnalyticalFunctionTriangleQuadrature(
    const ThreeChannelPositionMapEvaluator& evaluator)
    : m_three_channel_evaluator(evaluator)
{
    std::cout << "==== AnalyticalFunctionTriangleQuadrature constructor" << std::endl;
}

AnalyticalFunctionTriangleQuadrature::~AnalyticalFunctionTriangleQuadrature() = default;

double AnalyticalFunctionTriangleQuadrature::get_error_one_triangle_exact(
    const Vector2<double>& uv0,
    const Vector2<double>& uv1,
    const Vector2<double>& uv2) const
{
    return get_error_one_triangle_exact_T<double>(uv0, uv1, uv2);
}
DScalar AnalyticalFunctionTriangleQuadrature::get_error_one_triangle_exact(
    const Vector2<DScalar>& uv0,
    const Vector2<DScalar>& uv1,
    const Vector2<DScalar>& uv2) const
{
    return get_error_one_triangle_exact_T<DScalar>(uv0, uv1, uv2);
}
} // namespace function::utils
} // namespace wmtk::components