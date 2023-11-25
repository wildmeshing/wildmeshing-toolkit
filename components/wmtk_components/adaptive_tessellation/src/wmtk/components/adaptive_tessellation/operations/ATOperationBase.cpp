#include "ATOperationBase.hpp"

namespace wmtk::components::adaptive_tessellation::operations {
using namespace wmtk::operations;
ATOperationBase::ATOperationBase(
    TriMesh& position_mesh,
    const Tuple& t,
    const OperationSettings<ATOperationBase>& settings)
    : TriMeshOperation(position_mesh)
    , TupleOperation(settings.invariants, t)
{
    // settings.initialize_invariants(uv_mesh);
}
bool ATOperationBase::execute() {
    return true;
}
} // namespace wmtk::components::adaptive_tessellation::operations
