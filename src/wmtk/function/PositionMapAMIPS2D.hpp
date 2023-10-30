#pragma once
#include <wmtk/function/utils/PositionMapEvaluator.hpp>
#include "AMIPS.hpp"

namespace wmtk::function {
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
        const wmtk::image::SamplingAnalyticFunction::FunctionType type,
        const double a,
        const double b,
        const double c);

public:
    DScalar get_value_autodiff(const Tuple& simplex) const override;

protected:
    utils::PositionMapEvaluator m_pos_evaluator;
};
} // namespace wmtk::function
