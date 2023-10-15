
#pragma once
#include <wmtk/operations/tri_mesh/VertexSmoothUsingDifferentiableEnergy.hpp>

namespace wmtk::operations::tri_mesh::internal {
class VertexSmoothGradientDescent : public VertexSmoothUsingDifferentiableEnergy
{
public:
    VertexSmoothGradientDescent(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

protected:
    template <int Dim>
    Eigen::Vector<double, Dim> get_descent_direction(optimization::FunctionInterface<Dim>&) const;
    bool execute() override;
    std::string name() const;
};
} // namespace wmtk::operations::tri_mesh::internal
