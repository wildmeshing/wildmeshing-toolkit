#pragma once
#include "VertexSmoothUsingDifferentiableEnergy.hpp"

namespace wmtk::operations {

namespace tri_mesh {
class VertexSmoothNewtonMethod : public VertexSmoothUsingDifferentiableEnergy
{
public:
    VertexSmoothNewtonMethod(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

protected:
    bool execute() override;
    std::string name() const;
};
} // namespace tri_mesh
} // namespace wmtk::operations