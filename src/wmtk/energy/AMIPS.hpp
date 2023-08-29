
#include "DifferentiableEnergy.hpp"
namespace wmtk {
namespace energy {

class AMIPS_2D : public DifferentiableEnergy
{
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
    static T
    energy_eval(const Eigen::Vector2d& uv1, const Eigen::Vector2d& uv2, const Eigen::Vector2d& uv3);
};

/**
 * @brief TODO 3D AMIPS uses uv and displacement map to get the 3d cooridnates then evaluate
 *
 */
class AMIPS_3D : public DifferentiableEnergy
{
    using DScalar = DScalar2<double, Eigen::VectorXd, Eigen::MatrixXd>;

public:
    double energy_eval(const Tuple& tuple) const override;
    DScalar energy_eval_autodiff(const Tuple& tuple) const override;

    template <typename T>
    static T
    energy_eval(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);
};
} // namespace energy
} // namespace wmtk