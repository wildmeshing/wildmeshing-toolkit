#include "EdgeSplitAtMidpoint.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {

std::shared_ptr<InvariantCollection>
OperationSettings<tri_mesh::EdgeSplitAtMidpoint>::create_invariants()
{
    std::make_shared<InvariantCollection> inv_col_ptr(m_mesh);
    inv_col_ptr->add(split_settings.create_invariants());
    inv_col_ptr->add(
        std::make_shared<MinEdgeLengthInvariant>(m_mesh, position, min_squared_length));
    return inv_col_ptr;
}

bool OperationSettings<tri_mesh::EdgeSplitAtMidpoint>::are_invariants_initialized(
    std::shared_ptr<InvariantCollection> inv_col) const
{
    return find_invariants_in_collection_by_type<MinEdgeLengthInvariant>(*inv_col);
}

namespace tri_mesh {
EdgeSplitAtMidpoint::EdgeSplitAtMidpoint(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeSplitAtMidpoint>& settings)
    : EdgeSplit(m, t, settings.split_settings)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{
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
