#pragma once
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/function/utils/PositionMapEvaluator.hpp>
#include <wmtk/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include "AutodiffFunction.hpp"
#include "TriangleAutodiffFunction.hpp"
namespace image = wmtk::components::adaptive_tessellation::image;
namespace wmtk::function {

using DScalar = typename AutodiffFunction::DScalar;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;
/**
 * @brief This is the implementation of the AMIPS energy function of a triangle mesh that can be
 * embedded in 2d or 3d. It uses autodiff encoding for differentiations.
 *
 */
class PerTriangleTextureIntegralAccuracyFunction : public TriangleAutodiffFunction
{
public:
    PerTriangleTextureIntegralAccuracyFunction(
        const TriMesh& mesh,
        const MeshAttributeHandle<double>& vertex_uv_handle,
        const std::array<image::Image, 3>& images,
        const image::SAMPLING_METHOD sampling_method = image::SAMPLING_METHOD::Bicubic,
        const image::IMAGE_WRAPPING_MODE wrapping_mode = image::IMAGE_WRAPPING_MODE::MIRROR_REPEAT);


    ~PerTriangleTextureIntegralAccuracyFunction();


protected:
    utils::ThreeChannelPositionMapEvaluator m_pos_evaluator;


    DScalar eval(const Simplex& domain_simplex, const std::array<DSVec, 3>& coordinates)
        const override;
};

} // namespace wmtk::function
