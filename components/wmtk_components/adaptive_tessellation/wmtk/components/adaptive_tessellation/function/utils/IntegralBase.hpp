#pragma once
#include <wmtk/function/utils/autodiff.h>
#include <wmtk/components/adaptive_tessellation/image/utils/SamplingParameters.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
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

    virtual double get_error_one_triangle_exact(
        const Eigen::Vector2<double>& uv0,
        const Eigen::Vector2<double>& uv1,
        const Eigen::Vector2<double>& uv2) const
    {
        throw std::runtime_error("Not implemented");
    }
    virtual DScalar get_error_one_triangle_exact(
        const Eigen::Vector2<DScalar>& uv0,
        const Eigen::Vector2<DScalar>& uv1,
        const Eigen::Vector2<DScalar>& uv2) const
    {
        throw std::runtime_error("Not implemented");
    }

protected:
    image::SAMPLING_METHOD m_sampling_method = image::SAMPLING_METHOD::Bicubic;
    // IntegrationMethod m_integration_method = IntegrationMethod::Exact;
    QuadratureOrder m_quadrature_order = QuadratureOrder::Reduced;
};

} // namespace wmtk::components::function::utils
