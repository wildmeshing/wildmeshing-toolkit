#include "DifferentiableEnergy.hpp"

class AMIPS_2D : public wmtk::Energy
{
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;

public:
    double energy_eval(const Tuple& tuple) const override{};
    DScalar energy_eval_autodiff(const Tuple& tuple) const override{};

    /**
     * @brief gradient defined wrt the first vertex
     *
     * @param uv1
     * @param uv2
     * @param uv3
     * @return double energy value
     */
    static double
    energy_eval(const Eigen::Vector2d uv1, const Eigen::Vector2d uv2, const Eigen::Vector2d uv3){};

    static DScalar energy_eval_autodiff(
        const Eigen::Vector2d uv1,
        const Eigen::Vector2d uv2,
        const Eigen::Vector2d uv3){};
};

/**
 * @brief TODO 3D AMIPS uses uv and displacement map to get the 3d cooridnates then evaluate
 *
 */
class AMIPS_3D : public wmtk::Energy
{
public:
    double energy_eval(const Tuple& tuple) const override{};
    DScalar energy_eval_autodiff(const Tuple& tuple) const override{};

    static double
    energy_eval(const Eigen::Vector3d p1, const Eigen::Vector3d p2, const Eigen::Vector3d p3){};
    static DScalar energy_eval_autodiff(
        const Eigen::Vector3d p1,
        const Eigen::Vector3d p2,
        const Eigen::Vector3d p3){};
};