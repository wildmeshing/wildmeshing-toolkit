#include <wmtk/energy/utils/autodiff.h>
#include <wmtk/Tuple.hpp>
#include <wmtk/energy/utils/AutoDiffTypes.hpp>
#include "Energy.hpp"
namespace wmtk {
namespace energy {
using DScalar = DScalar2<double, Eigen::VectorXd, Eigen::MatrixXd>;
using Scalar = typename DScalar::Scalar;


class DifferentiableEnergy : public Energy
{
public:
    virtual DScalar energy_eval_autodiff(const Tuple& tuple) const = 0;
};
} // namespace energy
} // namespace wmtk
