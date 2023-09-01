#pragma once
#include <wmtk/energy/utils/DofsToPosition.hpp>
#include "DifferentiableEnergy.hpp"
namespace wmtk {
namespace energy {
class AMIPS : public DifferentiableEnergy
{
public:
    AMIPS(const TriMesh& mesh);

public:
    std::array<double, 6> m_target_triangle = {0., 0., 1., 0., 1. / 2., sqrt(3) / 2.};
    double m_target_edge_length = 1.;

    void set_target_triangle(const std::array<double, 6>& target_triangle)
    {
        m_target_triangle = target_triangle;
    }
    void set_target_edge_length(const double target_edge_length)
    {
        m_target_edge_length = target_edge_length;
    }
    void set_target_triangle_with_scaling(const double target_edge_length)
    {
        m_target_edge_length = target_edge_length;
        m_target_triangle[0] *= m_target_edge_length;
        m_target_triangle[1] *= m_target_edge_length;
        m_target_triangle[2] *= m_target_edge_length;
        m_target_triangle[3] *= m_target_edge_length;
        m_target_triangle[4] *= m_target_edge_length;
        m_target_triangle[5] *= m_target_edge_length;
    }
};

class AMIPS_2D : public AMIPS
{
public:
    AMIPS_2D(const TriMesh& mesh)
        : AMIPS(mesh)
    {}

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
    static T energy_eval(
        const Eigen::Vector2d& uv1,
        const Eigen::Vector2d& uv2,
        const Eigen::Vector2d& uv3,
        const std::array<double, 6>& m_target_triangle);
}; // namespace energy

/**
 * @brief TODO 3D AMIPS uses uv and displacement map to get the 3d cooridnates then evaluate
 *
 */
class AMIPS_3DEmbedded : public AMIPS
{
public:
    AMIPS_3DEmbedded(const TriMesh& mesh)
        : AMIPS(mesh)
    {}

public:
    double energy_eval(const Tuple& tuple) const override;
    DScalar energy_eval_autodiff(const Tuple& tuple) const override;

    template <typename T>
    static T energy_eval(
        const Eigen::Vector2d& uv1,
        const Eigen::Vector2d& uv2,
        const Eigen::Vector2d& uv3,
        const std::array<double, 6>& m_target_triangle);
};
} // namespace energy
} // namespace wmtk