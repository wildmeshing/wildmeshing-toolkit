#pragma once
#include <wmtk/function/utils/PositionMapEvaluator.hpp>
#include "AMIPS.hpp"
namespace image = wmtk::components::adaptive_tessellation::image;
namespace wmtk::function {
using DScalar = typename AutodiffFunction::DScalar;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;
/**
 * @brief 2D AMIPS uses uv and position map to get the 3d cooridnates then evaluate
 *
 */
class PositionMapAMIPS2D : public AMIPS
{
public:
    PositionMapAMIPS2D(
        const TriMesh& mesh,
        const MeshAttributeHandle<double>& vertex_uv_handle,
        const image::Image& image);
    PositionMapAMIPS2D(
        const TriMesh& mesh,
        const MeshAttributeHandle<double>& vertex_uv_handle,
        const image::SamplingAnalyticFunction::FunctionType type,
        const double a,
        const double b,
        const double c);

protected:
    utils::PositionMapEvaluator m_pos_evaluator;

    // input coordinates are uv coordinates
    DScalar eval(const Simplex& domain_simplex, const std::array<DSVec, 3>& coordinates)
        const override;
};
} // namespace wmtk::function
