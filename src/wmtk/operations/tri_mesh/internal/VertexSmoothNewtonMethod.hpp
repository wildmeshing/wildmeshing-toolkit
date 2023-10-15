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

protected:
    bool execute() override;
    template <int Dim>
    Eigen::Vector<double, Dim> get_descent_direction(optimization::FunctionInterface<Dim>&) const;
    std::string name() const;
};
} // namespace wmtk::operations::tri_mesh::internal
