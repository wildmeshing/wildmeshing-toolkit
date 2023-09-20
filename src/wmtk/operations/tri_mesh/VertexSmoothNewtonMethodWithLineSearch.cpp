#include "VertexSmoothNewtonMethodWithLineSearch.hpp"

namespace wmtk::operations {
namespace tri_mesh {
VertexSmoothNewtonMethodWithLineSearch::VertexSmoothNewtonMethodWithLineSearch(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothOptimization>& settings)
    : VertexSmoothNewTonMethod(m, t, settings)
{}

bool VertexSmoothNewtonMethodWithLineSearch::execute()
{
    Tuple smooth_ret;
    {
        OperationSettings<tri_mesh::VertexSmoothNewtonMethod> op_settings;
        op_settings.initialize_invariants(mesh());
        tri_mesh::VertexSmoothNewtonMethod smooth_op(mesh(), input_tuple(), op_settings);
        if (!smooth_op()) {
            // line search
            while () }
        smooth_ret = smooth_op.return_tuple();
    }

    return true;
}
} // namespace tri_mesh

} // namespace wmtk::operations