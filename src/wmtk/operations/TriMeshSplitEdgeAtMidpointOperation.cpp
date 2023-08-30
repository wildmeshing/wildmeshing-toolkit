#include "TriMeshSplitEdgeAtMidpointOperation.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include "TriMeshSplitEdgeOperation.hpp"
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>

namespace wmtk {

void OperationSettings<TriMeshSplitEdgeAtMidpointOperation>::initialize_invariants(const TriMesh& m)
{
    split_settings.initialize_invariants(m);
    split_settings.invariants.add(std::make_shared<MinEdgeLengthInvariant>(m, position, min_squared_length));
}

bool OperationSettings<TriMeshSplitEdgeAtMidpointOperation>::are_invariants_initialized() const
{
        return split_settings.are_invariants_initialized() && find_invariants_in_collection_by_type<MinEdgeLengthInvariant>(split_settings.invariants);
}
TriMeshSplitEdgeAtMidpointOperation::TriMeshSplitEdgeAtMidpointOperation(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshSplitEdgeAtMidpointOperation>& settings)
    : TupleOperation(m, settings.split_settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{
    p0 = m_pos_accessor.vector_attribute(input_tuple());
    p1 = m_pos_accessor.vector_attribute(m_mesh.switch_vertex(input_tuple()));
}
std::string TriMeshSplitEdgeAtMidpointOperation::name() const
{
    return "tri_mesh_split_edge_at_midpoint";
}
Tuple TriMeshSplitEdgeAtMidpointOperation::return_tuple() const
{
    return m_output_tuple;
}
bool TriMeshSplitEdgeAtMidpointOperation::before() const
{
    return TupleOperation::before();
    if (m_mesh.is_outdated(input_tuple()) || !m_mesh.is_valid(input_tuple())) {
        return false;
    }

    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared > m_settings.min_squared_length;
}
bool TriMeshSplitEdgeAtMidpointOperation::execute()
{
    {
        TriMeshSplitEdgeOperation split_op(m_mesh, input_tuple(), m_settings.split_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (p0 + p1);

    return true;
}
} // namespace wmtk
