#include <wmtk/utils/autodiff.h>
#include "Energy.hpp"
class DifferentiableEnergy : public Energy
{
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;

public:
    virtual DScalar energy_eval_autodiff(const Tuple& tuple) const = 0;
}