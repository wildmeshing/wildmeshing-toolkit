#include <wmtk/energy/utils/autodiff.h>
#include "Energy.hpp"
class DifferentiableEnergy : public Energy
{
    using DScalar = DScalar2<double, Eigen::VectorXd, Eigen::MatrixXd>;

public:
    virtual DScalar energy_eval_autodiff(const Tuple& tuple) const = 0;
}