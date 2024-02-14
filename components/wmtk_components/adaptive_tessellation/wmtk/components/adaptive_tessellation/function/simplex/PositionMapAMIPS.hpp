#pragma once
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/function/utils/PositionMapEvaluator.hpp>


namespace image = wmtk::components::image;
namespace wmtk::function {
using namespace wmtk::simplex;
using DScalar = typename wmtk::function::PerSimplexAutodiffFunction::DScalar;
using DSVec = typename wmtk::function::PerSimplexAutodiffFunction::DSVec;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;

class PositionMapAMIPS : public PerSimplexAutodiffFunction
{
public:
    PositionMapAMIPS(
        const Mesh& mesh,
        const attribute::MeshAttributeHandle& vertex_uv_handle,
        std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
            pos_evaluator_ptr,
        double amips_weight = 1,
        bool amips_area_weighted = false);

    ~PositionMapAMIPS();


protected:
    std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
        m_pos_evaluator_ptr;
    double m_amips_weight;
    bool m_amips_area_weighted;
    DScalar eval(const Simplex& domain_simplex, const std::vector<DSVec>& coordinates)
        const override;
};

} // namespace wmtk::function
