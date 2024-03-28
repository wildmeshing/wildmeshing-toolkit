#pragma once
#include <wmtk/function/utils/autodiff.h>
#include <wmtk/components/adaptive_tessellation/image/utils/SamplingParameters.hpp>
#include <wmtk/components/adaptive_tessellation/quadrature/Quadrature.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "Triangle2DTo3DMapping.hpp"
namespace image = wmtk::components::image;
namespace wmtk::components::function::utils {
using DScalar = typename wmtk::function::PerSimplexAutodiffFunction::DScalar;
struct QuadratureCache
{
    wmtk::Quadrature quad;
    wmtk::Quadrature tmp;
};

struct Cache
{
    // Data for exact error computation
    // mutable tbb::enumerable_thread_specific<QuadratureCache> quadrature_cache;
    QuadratureCache quadrature_cache;
};

class IntegralBasedAvgDistance : public Triangle2DTo3DMapping
{
public:
    // enum class IntegrationMethod { Exact, Adaptive };
    IntegralBasedAvgDistance(const ThreeChannelPositionMapEvaluator& evaluator)
        : Triangle2DTo3DMapping(evaluator)
    {}
    enum class QuadratureOrder { Full, Reduced };
    virtual ~IntegralBasedAvgDistance() = default;

public:
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

    double distance(
        const Vector2<double>& uv0,
        const Vector2<double>& uv1,
        const Vector2<double>& uv2) const override
    {
        double value = triangle_quadrature(uv0, uv1, uv2);
        value = value / wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
        return value;
    }

    DScalar distance(
        const Vector2<DScalar>& uv0,
        const Vector2<DScalar>& uv1,
        const Vector2<DScalar>& uv2) const override
    {
        DScalar value = triangle_quadrature(uv0, uv1, uv2);
        value = value / wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
        return value;
    }


protected:
    QuadratureOrder m_quadrature_order = QuadratureOrder::Reduced;
};

} // namespace wmtk::components::function::utils
