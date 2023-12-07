#include "EdgeSplitAtMidpoint.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {

void OperationSettings<tri_mesh::EdgeSplitAtMidpoint>::create_invariants()
{
    split_settings.create_invariants();

    invariants = std::make_shared<InvariantCollection>(m_mesh);
    invariants->add(std::make_shared<MinEdgeLengthInvariant>(m_mesh, position, min_squared_length));
}

namespace tri_mesh {
EdgeSplitAtMidpoint::EdgeSplitAtMidpoint(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<EdgeSplitAtMidpoint>& settings)
    : EdgeSplit(m, t, settings.split_settings)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Edge);
    coord0 = m_pos_accessor.vector_attribute(input_tuple());
    coord1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
}
std::string EdgeSplitAtMidpoint::name() const
{
    return "tri_mesh_split_edge_at_midpoint";
}
Tuple EdgeSplitAtMidpoint::return_tuple() const
{
    return m_output_tuple;
}
bool EdgeSplitAtMidpoint::before() const
{
    return TupleOperation::before();
}
bool EdgeSplitAtMidpoint::execute()
{
    if (!EdgeSplit::execute()) {
        return false;
    }
    m_output_tuple = EdgeSplit::return_tuple();
    m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (coord0 + coord1);

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
