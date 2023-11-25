#include "ATOperationBase.hpp"

namespace wmtk::components::adaptive_tessellation::operations {
using namespace wmtk::operations;
ATOperationBase::ATOperationBase(
    const TriMesh& uv_mesh,
    const TriMesh& position_mesh,
    const std::vector<EdgeMesh>& edge_meshes,
    const std::map<Mesh, Mesh>& edge_meshes_map,
    const Tuple& t,
    const OperationSettings<ATOperationBase>& settings)
    : TriMeshOperation(uv_mesh)
    , TupleOperation(settings.invariants, t)
    , m_position_mesh(position_mesh)
    , m_edge_meshes(edge_meshes)
    , m_sibling_meshes_map(edge_meshes_map)
{
    // settings.initialize_invariants(uv_mesh);
}
bool ATOperationBase::execute() {}
} // namespace wmtk::components::adaptive_tessellation::operations