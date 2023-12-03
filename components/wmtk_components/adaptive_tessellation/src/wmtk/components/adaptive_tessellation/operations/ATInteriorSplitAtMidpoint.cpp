#include "ATInteriorSplitAtMidpoint.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

namespace AT_op = wmtk::components::adaptive_tessellation::operations;

void wmtk::operations::OperationSettings<AT_op::ATInteriorSplitAtMidpoint>::initialize_invariants(
    const TriMesh& uv_mesh)
{
    m_AT_op.initialize_invariants();
    edge_split_midpoint_settings.initialize_invariants(uv_mesh);
    edge_split_midpoint_settings.position = m_AT_op.m_uv_handle;
    // TODO format this better. The min_squred_length should be read from an parameter file
    edge_split_midpoint_settings.min_squared_length = 0.1;
    assert(!split_boundary_edges);
    AT_interior_split_invariants.add(
        std::make_shared<wmtk::invariants::InteriorSimplexInvariant>(uv_mesh, PrimitiveType::Edge));
}
bool wmtk::operations::OperationSettings<
    AT_op::ATInteriorSplitAtMidpoint>::are_invariants_initialized() const
{
    return wmtk::find_invariants_in_collection_by_type<wmtk::invariants::InteriorSimplexInvariant>(
        AT_interior_split_invariants);
}

wmtk::operations::OperationSettings<AT_op::ATInteriorSplitAtMidpoint>::OperationSettings(
    AT_op::internal::ATOperation AT_op)
    : m_AT_op(AT_op)
{}

namespace wmtk::components::adaptive_tessellation::operations {
using namespace wmtk::operations;
ATInteriorSplitAtMidpoint::ATInteriorSplitAtMidpoint(
    Mesh& uv_mesh,
    const Tuple& t,
    const wmtk::operations::OperationSettings<ATInteriorSplitAtMidpoint>& settings)
    : ATInteriorSplitAtMidpoint(static_cast<TriMesh&>(uv_mesh), t, settings)
{}

ATInteriorSplitAtMidpoint::ATInteriorSplitAtMidpoint(
    TriMesh& uv_mesh,
    const Tuple& t,
    const wmtk::operations::OperationSettings<ATInteriorSplitAtMidpoint>& settings)
    : tri_mesh::EdgeSplitAtMidpoint(uv_mesh, t, settings.edge_split_midpoint_settings)
    , m_settings(settings)
    , m_pos_accessor(position_mesh().create_accessor(settings.m_AT_op.m_position_handle))
{
    assert(m_settings.are_invariants_initialized());
}

bool ATInteriorSplitAtMidpoint::execute()
{
    Tuple uv_input = input_tuple();
    assert(uv_mesh().is_valid_slow(uv_input));
    std::vector<Simplex> position_mesh_inputs =
        uv_mesh().map(position_mesh(), Simplex::edge(uv_input));
    assert(position_mesh_inputs.size() == 1);
    Tuple position_input = position_mesh_inputs[0].tuple();
    assert(position_mesh().is_valid_slow(position_input));
    pos0 = m_pos_accessor.vector_attribute(position_input);
    pos1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(position_input));

    if (!tri_mesh::EdgeSplitAtMidpoint::execute()) {
        return false;
    }
    // update the uv mesh coordinate attributes
    Tuple uv_new_vertex = tri_mesh::EdgeSplitAtMidpoint::return_tuple();
    TriMesh& uv_mesh = ATInteriorSplitAtMidpoint::uv_mesh();
    assert(uv_mesh.is_valid_slow(uv_new_vertex));
    TriMesh& position_mesh = ATInteriorSplitAtMidpoint::position_mesh();
    std::vector<Simplex> pos_new_vertices =
        uv_mesh.map(position_mesh, Simplex::vertex(uv_new_vertex));
    assert(pos_new_vertices.size() == 1);
    assert(position_mesh.is_valid_slow(pos_new_vertices[0].tuple()));

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
