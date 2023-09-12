#pragma once
#include <wmtk/image/Image.hpp>
#include "AutodiffFunction.hpp"
#include "utils/AutoDiffUtils.hpp"
#include "utils/DofsToPosition.hpp"
namespace wmtk {
namespace function {
class AMIPS : public AutodiffFunction
{
public:
    AMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);
    static Eigen::Matrix<double, 3, 2> get_target_triangle(double scaling);
};

class AMIPS_2D : public AMIPS
{
public:
    AMIPS_2D(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);

protected:
    DScalar get_value_autodiff(const Tuple& tuple) const override;

    /**
     * @brief gradient defined wrt the first vertex
     *
     * @param uv0
     * @param uv1
     * @param uv2
     * @return can be double or DScalar
     */
    template <typename T>
    T function_eval(
        const Eigen::Matrix<T, 2, 1>& uv0,
        const Eigen::Vector2d& uv1,
        const Eigen::Vector2d& uv2) const;
};

/**
 * @brief 3D AMIPS uses uv and displacement map to get the 3d cooridnates then evaluate
 *
 */
class AMIPS_3DEmbedded : public AMIPS
{
public:
    AMIPS_3DEmbedded(
        const TriMesh& mesh,
        const MeshAttributeHandle<double>& vertex_uv_handle,
        const image::Image& image);
    AMIPS_3DEmbedded(
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
} // namespace function
} // namespace wmtk
