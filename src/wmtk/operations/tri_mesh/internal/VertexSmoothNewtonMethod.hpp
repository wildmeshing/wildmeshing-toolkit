#pragma once
#include <wmtk/operations/tri_mesh/VertexSmoothUsingDifferentiableEnergy.hpp>

namespace wmtk::operations::tri_mesh::internal {

class VertexSmoothNewtonMethod : public VertexSmoothUsingDifferentiableEnergy
{
public:
    VertexSmoothNewtonMethod(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

    std::vector<double> priority() const override;

protected:
    bool execute() override;
    Eigen::VectorXd get_descent_direction(function::utils::DifferentiableFunctionEvaluator&) const;
    std::string name() const override;
};
} // namespace wmtk::operations::tri_mesh::internal
