#pragma once
#include <wmtk/function/utils/autodiff.h>
#include <wmtk/components/adaptive_tessellation/image/utils/SamplingParameters.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
#include <wmtk/utils/triangle_areas.hpp>
namespace image = wmtk::components::image;
namespace wmtk::components::function::utils {
using DScalar = typename wmtk::function::PerSimplexAutodiffFunction::DScalar;
class IntegralBase
{
public:
    // enum class IntegrationMethod { Exact, Adaptive };

    enum class QuadratureOrder { Full, Reduced };
    virtual ~IntegralBase() = default;

public:
    void set_sampling_method(image::SAMPLING_METHOD method) { m_sampling_method = method; }
    // void set_integration_method(IntegrationMethod method) { m_integration_method = method; }
    void set_quadrature_order(QuadratureOrder order) { m_quadrature_order = order; }

    virtual double triangle_quadrature(
        const Eigen::Vector2<double>& uv0,
        const Eigen::Vector2<double>& uv1,
        const Eigen::Vector2<double>& uv2) const
    {
        throw std::runtime_error("Not implemented");
    }
    virtual DScalar triangle_quadrature(
        const Eigen::Vector2<DScalar>& uv0,
        const Eigen::Vector2<DScalar>& uv1,
        const Eigen::Vector2<DScalar>& uv2) const
    {
        throw std::runtime_error("Not implemented");
    }
    template <typename T>
    T average_area_integral_over_triangle(
        const Vector2<T>& uv0,
        const Vector2<T>& uv1,
        const Vector2<T>& uv2) const
    {
        T value = triangle_quadrature(uv0, uv1, uv2);
        value = value / wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
        return value;
    }

protected:
    image::SAMPLING_METHOD m_sampling_method = image::SAMPLING_METHOD::Bicubic;
    // IntegrationMethod m_integration_method = IntegrationMethod::Exact;
    QuadratureOrder m_quadrature_order = QuadratureOrder::Reduced;
};

} // namespace wmtk::components::function::utils
