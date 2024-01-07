#pragma once
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
#include <wmtk/function/utils/PositionMapEvaluator.hpp>

namespace image = wmtk::components::adaptive_tessellation::image;
namespace wmtk::function {
using namespace wmtk::simplex;
using DScalar = typename wmtk::function::PerSimplexAutodiffFunction::DScalar;
using DSVec = typename wmtk::function::PerSimplexAutodiffFunction::DSVec;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;

class PerTriangleAnalyticalIntegral : public wmtk::function::PerSimplexAutodiffFunction
{
public:
    PerTriangleAnalyticalIntegral(
        const Mesh& mesh,
        const attribute::MeshAttributeHandle& vertex_uv_handle,
        const std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3> funcs,
        const image::SAMPLING_METHOD sampling_method = image::SAMPLING_METHOD::Analytical);

    ~PerTriangleAnalyticalIntegral();


protected:
    wmtk::components::adaptive_tessellation::function::utils::ThreeChannelPositionMapEvaluator
        m_pos_evaluator;
    DScalar eval(const Simplex& domain_simplex, const std::vector<DSVec>& coordinates)
        const override;
};

} // namespace wmtk::function
