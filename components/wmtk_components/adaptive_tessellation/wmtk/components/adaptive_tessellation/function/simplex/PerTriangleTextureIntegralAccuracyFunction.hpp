#pragma once
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
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
class PerTriangleTextureIntegralAccuracyFunction : public wmtk::function::PerSimplexAutodiffFunction
{
public:
    PerTriangleTextureIntegralAccuracyFunction(
        const Mesh& mesh,
        const attribute::MeshAttributeHandle& vertex_uv_handle,
        std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
            pos_evaluator_ptr);


    ~PerTriangleTextureIntegralAccuracyFunction();


protected:
    std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
        m_pos_evaluator_ptr;
    DScalar eval(const Simplex& domain_simplex, const std::vector<DSVec>& coordinates)
        const override;
};

} // namespace wmtk::function
