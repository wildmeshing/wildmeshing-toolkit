#include "ATSplit.hpp"

namespace wmtk::operations {
namespace tri_mesh {
ATSplit::ATSplit(
    const TriMesh& uv_mesh,
    const TriMesh& position_mesh,
    const std::vector<EdgeMesh>& edge_meshes,
    const std::map<Mesh, Mesh>& edge_meshes_map,
    const Tuple& t,
    const OperationSettings<ATSplit>& settings)
    : AT(uv_mesh, position_mesh, edge_meshes, edge_meshes_map, t, settings)
{
    settings.initialize_invariants(m);
}
bool ATSplit::execute()
{
    if (!EdgeSplitAtMidpoint::execute()) {
        return false;
    }
    // update the uv mesh coordinate attributes
    Tuple new_vertex = new_vertex();
    std::vector<Simplex> position_new_vertex = map(position_mesh(), new_vertex);

    return true;
}

Mesh ATSplit::position_mesh()
{
    return m_position_mesh;
}
} // namespace tri_mesh
} // namespace wmtk::operations