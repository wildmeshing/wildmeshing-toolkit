#pragma once

namespace wmtk {

class IntegralBase
{
public:
    enum class SamplingMethod {
        Nearest,
        Bilinear,
        Bicubic,
    };

    enum class IntegrationMethod { Exact, Adaptive };

    enum class QuadratureOrder { Full, Reduced };

public:
    void set_sampling_method(SamplingMethod method) { m_sampling_method = method; }
    void set_integration_method(IntegrationMethod method) { m_integration_method = method; }
    void set_quadrature_order(QuadratureOrder order) { m_quadrature_order = order; }

protected:
    SamplingMethod m_sampling_method = SamplingMethod::Bicubic;
    IntegrationMethod m_integration_method = IntegrationMethod::Exact;
    QuadratureOrder m_quadrature_order = QuadratureOrder::Reduced;
};

} // namespace wmtk
