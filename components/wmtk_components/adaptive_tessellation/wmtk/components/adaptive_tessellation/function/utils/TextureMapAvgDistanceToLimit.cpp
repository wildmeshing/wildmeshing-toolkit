#include "TextureMapAvgDistanceToLimit.hpp"

#include <wmtk/components/adaptive_tessellation/quadrature/ClippedQuadrature.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "BarycentricTriangle.hpp"

using namespace Eigen;
namespace wmtk::components {
namespace function::utils {

TextureMapAvgDistanceToLimit::TextureMapAvgDistanceToLimit(
    const ThreeChannelPositionMapEvaluator& evaluator)
    : IntegralBasedAvgDistance(evaluator)
    , m_cache(std::make_shared<Cache>())
{
    std::cout << "==== TextureMapAvgDistanceToLimit constructor" << std::endl;
}

TextureMapAvgDistanceToLimit::TextureMapAvgDistanceToLimit(
    const ThreeChannelPositionMapEvaluator& evaluator,
    bool debug)
    : IntegralBasedAvgDistance(evaluator)
    , m_cache(std::make_shared<Cache>())
    , m_debug(debug)
{
    if (m_debug) {
        std::cout << "==== TextureMapAvgDistanceToLimit with debug" << std::endl;
        m_jsonData_bary_coord = nlohmann::ordered_json();
        m_jsonData_texture_coord = nlohmann::ordered_json();
    }
}

TextureMapAvgDistanceToLimit::~TextureMapAvgDistanceToLimit() = default;

std::pair<int, double> TextureMapAvgDistanceToLimit::pixel_num_size_of_uv_triangle(
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

std::pair<int, double> TextureMapAvgDistanceToLimit::pixel_size_of_uv_triangle(
    int pixel_num,
    Eigen::AlignedBox2d& bbox) const
{
    assert(pixel_num > 0);
    double pixel_size = bbox.diagonal().maxCoeff() / pixel_num;
    return {pixel_num, pixel_size};
}

double TextureMapAvgDistanceToLimit::triangle_quadrature(
    const Vector2<double>& uv0,
    const Vector2<double>& uv1,
    const Vector2<double>& uv2) const
{
    return triangle_quadrature_T<double>(uv0, uv1, uv2);
}
DScalar TextureMapAvgDistanceToLimit::triangle_quadrature(
    const Vector2<DScalar>& uv0,
    const Vector2<DScalar>& uv1,
    const Vector2<DScalar>& uv2) const
{
    return triangle_quadrature_T<DScalar>(uv0, uv1, uv2);
}

} // namespace function::utils
} // namespace wmtk::components