#pragma once
#include "VertexSmoothNewtonMethod.hpp"
namespace wmtk::operations::tri_mesh::internal {
class VertexSmoothNewtonMethodWithLineSearch : public VertexSmoothNewtonMethod
{
public:
    VertexSmoothNewtonMethodWithLineSearch(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

protected:
    bool execute() override;
};
} // namespace wmtk::operations::tri_mesh::internal
