#pragma once
#include <wmtk/energy/utils/autodiff.h>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
// #include <wmtk/energy/utils/AutoDiffTypes.hpp>
#include "Energy.hpp"
namespace wmtk {
namespace energy {

using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>>;
using Scalar = typename DScalar::Scalar;

class DifferentiableEnergy : public Energy
{
public:
    DifferentiableEnergy(const Mesh& mesh)
        : Energy(mesh){};

    virtual ~DifferentiableEnergy() = default;

public:
    virtual DScalar energy_eval_autodiff(const Tuple& tuple) const = 0;
};
} // namespace energy
} // namespace wmtk
