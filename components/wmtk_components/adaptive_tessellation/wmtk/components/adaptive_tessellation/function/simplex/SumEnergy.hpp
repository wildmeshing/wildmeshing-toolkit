#pragma once
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include "PerTriangleAnalyticalIntegral.hpp"
#include "PerTriangleTextureIntegralAccuracyFunction.hpp"
#include "PositionMapAMIPS.hpp"

namespace image = wmtk::components::image;
namespace wmtk::function {
using namespace wmtk::simplex;
using DScalar = typename wmtk::function::PerSimplexAutodiffFunction::DScalar;
using DSVec = typename wmtk::function::PerSimplexAutodiffFunction::DSVec;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;
/**
 * @brief This is the implementation of the AMIPS energy function of a triangle mesh that can be
 * embedded in 2d or 3d. It uses autodiff encoding for differentiations.
 *
 */
class SumEnergy : public wmtk::function::PerSimplexAutodiffFunction
{
public:
    SumEnergy(
        const Mesh& mesh,
        const attribute::MeshAttributeHandle& vertex_uv_handle,
        wmtk::components::function::utils::ThreeChannelPositionMapEvaluator pos_evaluator,
        const double weight_lambda = 1e-3,
        const double barrier_area_constant = 1e-6,
        const image::SAMPLING_METHOD sampling_method = image::SAMPLING_METHOD::Bicubic);


    ~SumEnergy();


protected:
    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator m_pos_evaluator;
    double m_weight_lambda;
    double m_barrier_area;
    DScalar eval(const Simplex& domain_simplex, const std::vector<DSVec>& coordinates)
        const override;
};

} // namespace wmtk::function
