#include "ATOperationBase.hpp"
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
namespace wmtk::components::adaptive_tessellation::operations {
using namespace wmtk::operations;

void wmtk::operations::OperationSettings<ATOperationBase>::initialize_invariants(
    const Mesh& mesh,
    const TriMesh& uv_mesh)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(mesh);
    invariants.add(std::make_shared<TriangleInversionInvariant>(uv_mesh, uv));
}
ATOperationBase::ATOperationBase(
    Mesh& mesh,
    const Tuple& t,
    const OperationSettings<ATOperationBase>& settings)
    : TriMeshOperation(mesh)
    , TupleOperation(settings.invariants, t)
{}
} // namespace wmtk::components::adaptive_tessellation::operations
