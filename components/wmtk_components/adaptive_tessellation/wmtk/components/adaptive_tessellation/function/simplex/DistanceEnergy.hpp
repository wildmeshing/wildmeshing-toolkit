#pragma once
#include <wmtk/components/adaptive_tessellation/function/utils/IntegralBasedAvgDistance.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
#include <wmtk/function/utils/PositionMapEvaluator.hpp>

namespace image = wmtk::components::image;
namespace wmtk::function {
using namespace wmtk::simplex;

using DScalar = typename PerSimplexAutodiffFunction::DScalar;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;
/**
 * @brief This is the implementation of the AMIPS energy function of a triangle mesh that can be
 * embedded in 2d or 3d. It uses autodiff encoding for differentiations.
 *
 */
class DistanceEnergy : public wmtk::function::PerSimplexAutodiffFunction
{
public:
    DistanceEnergy(
        const Mesh& mesh,
        const attribute::MeshAttributeHandle& vertex_uv_handle,
        std::shared_ptr<wmtk::components::function::utils::IntegralBasedAvgDistance> integral_ptr,
        double weight = 1);

    ~DistanceEnergy();


protected:
    std::shared_ptr<wmtk::components::function::utils::IntegralBasedAvgDistance> m_integral_ptr;
    double m_weight;
    DScalar eval(const Simplex& domain_simplex, const std::vector<DSVec>& coordinates)
        const override;
};

} // namespace wmtk::function
