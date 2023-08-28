#pragma once
#include <wmtk/image/Image.hpp>
#include "DifferentiableEnergy.hpp"
#include "utils/AutoDiffUtils.hpp"
#include "utils/DofsToPosition.hpp"
namespace wmtk {
namespace energy {
class AMIPS : public DifferentiableEnergy
{
public:
    AMIPS(const TriMesh& mesh);
    Eigen::Matrix<double, 3, 2> get_target_triangle(double scaling) const;
};

class AMIPS_2D : public AMIPS
{
public:
    AMIPS_2D(const TriMesh& mesh);

public:
    double energy_eval(const Tuple& tuple) const override;
    DScalar energy_eval_autodiff(const Tuple& tuple) const override;

    /**
     * @brief gradient defined wrt the first vertex
     *
     * @param uv1
     * @param uv2
     * @param uv3
     * @return can be double or DScalar
     */
    template <typename T>
    T energy_eval(
        const Eigen::Vector2d& uv1,
        const Eigen::Vector2d& uv2,
        const Eigen::Vector2d& uv3) const;
}; // namespace energy

/**
 * @brief 3D AMIPS uses uv and displacement map to get the 3d cooridnates then evaluate
 *
 */
class AMIPS_3DEmbedded : public AMIPS
{
public:
    AMIPS_3DEmbedded(const TriMesh& mesh, const image::Image& image)
        : AMIPS(mesh)
        , m_dofs_to_pos(image)
    {}
    AMIPS_3DEmbedded(
        const TriMesh& mesh,
        const wmtk::image::SamplingAnalyticFunction::FunctionType type,
        const double a,
        const double b,
        const double c)
        : AMIPS(mesh)
        , m_dofs_to_pos(type, a, b, c)
    {}


public:
    double energy_eval(const Tuple& tuple) const override;
    DScalar energy_eval_autodiff(const Tuple& tuple) const override;

    template <typename T>
    T energy_eval(
        const Eigen::Matrix<T, Eigen::Dynamic, 1>& dofT,
        const Eigen::Vector2d& uv2,
        const Eigen::Vector2d& uv3) const;

protected:
    DofsToPosition m_dofs_to_pos;
};
} // namespace energy
} // namespace wmtk
