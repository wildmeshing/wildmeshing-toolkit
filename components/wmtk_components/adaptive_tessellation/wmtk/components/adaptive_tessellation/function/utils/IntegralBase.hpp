#pragma once
#include <wmtk/components/adaptive_tessellation/image/utils/SamplingParameters.hpp>
namespace wmtk {
namespace image = wmtk::components::image;
class IntegralBase
{
public:
    // enum class IntegrationMethod { Exact, Adaptive };

    enum class QuadratureOrder { Full, Reduced };

public:
    void set_sampling_method(image::SAMPLING_METHOD method) { m_sampling_method = method; }
    // void set_integration_method(IntegrationMethod method) { m_integration_method = method; }
    void set_quadrature_order(QuadratureOrder order) { m_quadrature_order = order; }

protected:
    image::SAMPLING_METHOD m_sampling_method = image::SAMPLING_METHOD::Bicubic;
    // IntegrationMethod m_integration_method = IntegrationMethod::Exact;
    QuadratureOrder m_quadrature_order = QuadratureOrder::Reduced;
};

} // namespace wmtk
