#pragma once
#include <wmtk/components/adaptive_tessellation/function/utils/IntegralBase.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/function/utils/PositionMapEvaluator.hpp>

namespace image = wmtk::components::image;
namespace wmtk::function {
using namespace wmtk::simplex;

/**
 * @brief This is the implementation of the AMIPS energy function of a triangle mesh that can be
 * embedded in 2d or 3d. It uses autodiff encoding for differentiations.
 *
 */
class DistanceEnergyNonDiff : public wmtk::function::PerSimplexFunction
{
public:
    DistanceEnergyNonDiff(
        const Mesh& mesh,
        const attribute::MeshAttributeHandle& vertex_uv_handle,
        std::shared_ptr<wmtk::components::function::utils::IntegralBase> integral_ptr,
        double weight = 1);

    ~DistanceEnergyNonDiff();


protected:
    std::shared_ptr<wmtk::components::function::utils::IntegralBase> m_integral_ptr;
    double m_weight;
    double get_value(const Simplex& domain_simplex) const override;
};

} // namespace wmtk::function
