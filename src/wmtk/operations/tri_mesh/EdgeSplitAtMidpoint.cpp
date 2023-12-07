#include "EdgeSplitAtMidpoint.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {

void OperationSettings<tri_mesh::EdgeSplitAtMidpoint>::create_invariants()
{
    OperationSettings<tri_mesh::EdgeSplit>::create_invariants();

    invariants->add(std::make_shared<MinEdgeLengthInvariant>(m_mesh, position, min_squared_length));
}

namespace tri_mesh {
EdgeSplitAtMidpoint::EdgeSplitAtMidpoint(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<EdgeSplitAtMidpoint>& settings)
    : EdgeSplit(m, t, settings)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Edge);
}
std::string EdgeSplitAtMidpoint::name() const
{
    return "tri_mesh_split_edge_at_midpoint";
}
bool EdgeSplitAtMidpoint::before() const
{
    return TupleOperation::before();
}
bool EdgeSplitAtMidpoint::execute()
{
    Eigen::VectorXd coord0 = m_pos_accessor.vector_attribute(input_tuple());
    Eigen::VectorXd coord1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
    if (!EdgeSplit::execute()) {
        return false;
    }
    m_pos_accessor.vector_attribute(EdgeSplit::return_tuple()) = 0.5 * (coord0 + coord1);

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
