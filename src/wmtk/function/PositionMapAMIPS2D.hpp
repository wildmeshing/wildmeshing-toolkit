#pragma once
#include "AMIPS.hpp"

/**
 * @brief 2D AMIPS uses uv and position map to get the 3d cooridnates then evaluate
 *
 */
class PositionMapAMIPS2D : public AMIPS2D
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
    DScalar get_value_autodiff(const Tuple& tuple) const override;

protected:
protected:
    DofsToPosition m_dofs_to_pos;
};
