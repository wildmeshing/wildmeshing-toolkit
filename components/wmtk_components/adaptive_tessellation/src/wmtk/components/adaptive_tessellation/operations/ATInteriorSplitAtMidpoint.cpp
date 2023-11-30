#include "ATInteriorSplitAtMidpoint.hpp"

namespace wmtk::operations {
namespace tri_mesh {
ATInteriorSplitAtMidpoint::ATInteriorSplitAtMidpoint(
    const TriMesh& position_mesh,
    const Tuple& t,
    const OperationSettings<ATInteriorSplitAtMidpoint>& settings)
    : EdgeSplitMidAtMidpoint(position_mesh, t, settings.edge_split_midpoint_settings)
    , m_uv_accessor{uv_mesh().create_accessor(settings.uv)}
    , m_settings{settings}
{
    settings.initialize_invariants(m);
    uv0 = m_uv_accessor.vector_attribute(input_tuple());
    uv1 = m_uv_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
}
bool ATInteriorSplitAtMidpoint::execute()
{
    if (!EdgeSplitAtMidpoint::execute()) {
        return false;
    }
    // update the uv mesh coordinate attributes
    Tuple position_new_vertex = new_vertex();
    Tuple uv_new_vertex = position_mesh().map(uv_mesh(), position_new_vertex);
    m_uv_accessor.vector_attribute(uv_new_vertex) = 0.5 * (u0 + u1);

    return true;
}

const TriMesh& ATInteriorSplitAtMidpoint::position_mesh() const
{
    return settings.base_settings.position_mesh;
}
const TriMesh& ATInteriorSplitAtMidpoint::uv_mesh() const
{
    return settings.base_settings.uv_mesh;
}
} // namespace tri_mesh
} // namespace wmtk::operations