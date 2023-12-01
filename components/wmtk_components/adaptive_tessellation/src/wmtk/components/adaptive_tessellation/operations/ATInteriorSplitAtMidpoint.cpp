#include "ATInteriorSplitAtMidpoint.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>

namespace AT_op = wmtk::components::adaptive_tessellation::operations;

void wmtk::operations::OperationSettings<AT_op::ATInteriorSplitAtMidpoint>::initialize_invariants(
    const TriMesh& uv_mesh)
{
    m_AT_op.initialize_invariants(uv_mesh, uv_mesh);
    edge_split_midpoint_settings.initialize_invariants(uv_mesh);
    assert(!split_boundary_edges);
    AT_interior_split_invariants.add(
        std::make_shared<wmtk::invariants::InteriorSimplexInvariant>(uv_mesh, PrimitiveType::Edge));
}
namespace wmtk::components::adaptive_tessellation::operations {
using namespace wmtk::operations;

ATInteriorSplitAtMidpoint::ATInteriorSplitAtMidpoint(
    const Tuple& t,
    const wmtk::operations::OperationSettings<ATInteriorSplitAtMidpoint>& settings)
    : tri_mesh::EdgeSplitAtMidpoint(uv_mesh(), t, settings.edge_split_midpoint_settings)
    , m_pos_accessor(position_mesh().create_accessor(settings.m_pos_handle))
    , m_settings(settings)
{
    pos0 = m_pos_accessor.vector_attribute(input_tuple());
    pos1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
}

bool ATInteriorSplitAtMidpoint::execute()
{
    if (!tri_mesh::EdgeSplitAtMidpoint::execute()) {
        return false;
    }
    // update the uv mesh coordinate attributes
    Tuple uv_new_vertex = tri_mesh::EdgeSplitAtMidpoint::return_tuple();
    TriMesh& uv_mesh = ATInteriorSplitAtMidpoint::uv_mesh();
    TriMesh& position_mesh = ATInteriorSplitAtMidpoint::position_mesh();
    std::vector<Simplex> pos_new_vertices =
        uv_mesh.map(position_mesh, Simplex::vertex(uv_new_vertex));
    assert(pos_new_vertices.size() == 1);
    m_pos_accessor.vector_attribute(pos_new_vertices[0].tuple()) = 0.5 * (pos0 + pos1);

    return true;
}

TriMesh& ATInteriorSplitAtMidpoint::uv_mesh()
{
    return *m_settings.m_AT_op.uv_mesh_ptr().get();
}
TriMesh& ATInteriorSplitAtMidpoint::position_mesh()
{
    return *m_settings.m_AT_op.position_mesh_ptr().get();
}

const TriMesh& ATInteriorSplitAtMidpoint::const_uv_mesh() const
{
    return *m_settings.m_AT_op.uv_mesh_ptr().get();
}
const TriMesh& ATInteriorSplitAtMidpoint::const_position_mesh() const
{
    return *m_settings.m_AT_op.position_mesh_ptr().get();
}

} // namespace wmtk::components::adaptive_tessellation::operations