#include "PositionMapEvaluator.hpp"
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/components/adaptive_tessellation/image/bicubic_interpolation.hpp>
#include <wmtk/function/AutodiffFunction.hpp>

namespace image = wmtk::components::adaptive_tessellation::image;
namespace wmtk::function::utils {

PositionMapEvaluator::PositionMapEvaluator() = default;
PositionMapEvaluator::~PositionMapEvaluator() = default;
PositionMapEvaluator::PositionMapEvaluator(PositionMapEvaluator&&) =
    default; // move assignment operator
PositionMapEvaluator& PositionMapEvaluator::operator=(PositionMapEvaluator&&) =
    default; // move assignment operator

/**
 * @brief Construct a new Dofs To Position object using a displacement map (requires a
 * sampler)
 *
 * @param image
 */
PositionMapEvaluator::PositionMapEvaluator(const image::Image& image)
{
    m_sampling = std::make_unique<image::SamplingBicubic>(image);
}

PositionMapEvaluator::PositionMapEvaluator(
    const image::SamplingAnalyticFunction::FunctionType type,
    const double a,
    const double b,
    const double c)
{
    m_sampling = std::make_unique<image::SamplingAnalyticFunction>(type, a, b, c);
}
/*
template <typename T>
Vector3<T> PositionMapEvaluator::uv_to_pos(const Vector2<T>& uv) const
{
    return Vector3<T>(uv.x(), uv.y(), m_sampling->sample(uv.x(), uv.y()));
}

template <>
Vector3<double> PositionMapEvaluator::uv_to_pos(const Vector2<double>& uv) const;


template <>
auto PositionMapEvaluator::uv_to_pos(const Vector2<wmtk::function::AutodiffFunction::DScalar>& uv)
    const -> Vector3<wmtk::function::AutodiffFunction::DScalar>;
    */
} // namespace wmtk::function::utils
