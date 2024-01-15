#include "TextureIntegral.hpp"

#include <wmtk/components/adaptive_tessellation/quadrature/ClippedQuadrature.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "BarycentricTriangle.hpp"

using namespace Eigen;
namespace wmtk::components {
namespace function::utils {

TextureIntegral::TextureIntegral(const ThreeChannelPositionMapEvaluator& evaluator)
    : m_three_channel_evaluator(evaluator)
    , m_cache(std::make_shared<Cache>())
{
    std::cout << "==== TextureIntegral constructor" << std::endl;
}

TextureIntegral::~TextureIntegral() = default;

std::pair<int, double> TextureIntegral::pixel_num_size_of_uv_triangle(
    Eigen::AlignedBox2d& bbox) const
{
    Vector2d bbox_min = bbox.min();
    Vector2d bbox_max = bbox.max();
    auto [xx1, yy1] = m_three_channel_evaluator.pixel_index(bbox_min);
    auto [xx2, yy2] = m_three_channel_evaluator.pixel_index(bbox_max);

    int num_pixels = std::max(abs(xx2 - xx1), abs(yy2 - yy1)) + 1;
    assert(num_pixels > 0);
    double pixel_size = bbox.diagonal().maxCoeff() / num_pixels;
    return {num_pixels, pixel_size};
}
double TextureIntegral::get_error_one_triangle_exact(
    const Vector2<double>& uv0,
    const Vector2<double>& uv1,
    const Vector2<double>& uv2) const
{
    return get_error_one_triangle_exact_T<double>(uv0, uv1, uv2);
}
DScalar TextureIntegral::get_error_one_triangle_exact(
    const Vector2<DScalar>& uv0,
    const Vector2<DScalar>& uv1,
    const Vector2<DScalar>& uv2) const
{
    return get_error_one_triangle_exact_T<DScalar>(uv0, uv1, uv2);
}
} // namespace function::utils
} // namespace wmtk::components