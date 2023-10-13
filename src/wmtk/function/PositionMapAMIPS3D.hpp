#pragma once
#include "AMIPS.hpp"

/**
 * @brief 3D AMIPS uses uv and position map to get the 3d cooridnates then evaluate
 *
 */
class PositionMapAMIPS3D : public AMIPS
{
public:
    PositionMapAMIPS3D(
        const TriMesh& mesh,
        const MeshAttributeHandle<double>& vertex_uv_handle,
        const image::Image& image);
    PositionMapAMIPS3D(
        const TriMesh& mesh,
        const MeshAttributeHandle<double>& vertex_uv_handle,
        const wmtk::image::SamplingAnalyticFunction::FunctionType type,
        const double a,
        const double b,
        const double c);

public:
    DScalar get_value_autodiff(const Tuple& tuple) const override;

protected:
    template <typename T>
    T function_eval(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& dofT,
        const Eigen::Vector2d& uv2,
        const Eigen::Vector2d& uv3) const;

protected:
    DofsToPosition m_dofs_to_pos;
};
