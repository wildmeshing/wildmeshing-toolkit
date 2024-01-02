
#pragma once
#include <wmtk/operations/tri_mesh/VertexSmoothUsingDifferentiableEnergy.hpp>

namespace wmtk::operations::tri_mesh::internal {
class VertexSmoothGradientDescent : public VertexSmoothUsingDifferentiableEnergy
{
public:
    VertexSmoothGradientDescent(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

    std::vector<double> priority() const override;

protected:
    Eigen::VectorXd get_descent_direction(function::utils::DifferentiableFunctionEvaluator&) const;
    bool execute() override;
    std::string name() const override;
};
} // namespace wmtk::operations::tri_mesh::internal