#include "VertexSmoothNewtonMethod.hpp"
namespace wmtk::operations {
namespace tri_mesh {
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
} // namespace tri_mesh
} // namespace wmtk::operations