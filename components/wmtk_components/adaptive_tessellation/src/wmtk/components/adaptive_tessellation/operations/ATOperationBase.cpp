#include "ATOperationBase.hpp"
#include <wmtk/invariants/TriangleInversionInvariant.hpp>

namespace AT_op = wmtk::components::adaptive_tessellation::operations;
void wmtk::operations::OperationSettings<AT_op::ATOperationBase>::initialize_invariants(
    const Mesh& mesh,
    const TriMesh& uv_mesh)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(mesh);
    invariants.add(std::make_shared<TriangleInversionInvariant>(uv_mesh, uv));
}
namespace wmtk::components::adaptive_tessellation::operations {
using namespace wmtk::operations;

ATOperationBase::ATOperationBase(
    Mesh& mesh,
    const Tuple& t,
    const OperationSettings<ATOperationBase>& settings)
    : TriMeshOperation(mesh)
    , TupleOperation(settings.invariants, t)
{}
} // namespace wmtk::components::adaptive_tessellation::operations
