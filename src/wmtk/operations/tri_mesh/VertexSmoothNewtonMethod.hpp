#include "VertexSmoothUsingDifferentiableEnergy.hpp"

namespace wmtk::operations::tri_mesh {

class VertexSmoothNewtonMethod : public VertexSmoothUsingDifferentiableEnergy
{
public:
    VertexSmoothNewtonMethod(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

protected:
    bool execute() override;
};
} // namespace wmtk::operations::tri_mesh