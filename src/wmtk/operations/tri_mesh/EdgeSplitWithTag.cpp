#include "EdgeSplitWithTag.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {


void OperationSettings<tri_mesh::EdgeSplitWithTag>::initialize_invariants(const TriMesh& m)
{
    split_settings.initialize_invariants(m);
    split_settings.invariants.add(
        std::make_shared<MinEdgeLengthInvariant>(m, position, min_squared_length));
}

bool OperationSettings<tri_mesh::EdgeSplitWithTag>::are_invariants_initialized() const
{
    return split_settings.are_invariants_initialized() &&
           find_invariants_in_collection_by_type<MinEdgeLengthInvariant>(split_settings.invariants);
}
namespace tri_mesh {
EdgeSplitWithTag::EdgeSplitWithTag(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeSplitWithTag>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.split_settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_tag_accessor{m.create_accessor(settings.tag)}
    , m_settings{settings}
{}
std::string EdgeSplitWithTag::name() const
{
    return "tri_mesh_split_edge_at_midpoint";
}
Tuple EdgeSplitWithTag::return_tuple() const
{
    return m_output_tuple;
}
bool EdgeSplitWithTag::before() const
{
    long tag0 = m_tag_accessor.vector_attribute(input_tuple())(0);
    long tag1 = m_tag_accessor.vector_attribute(mesh().switch_vertex(input_tuple()))(0);
    if (m_settings.split_when_tags == TAGS_DIFFERENT) {
        if (tag0 == m_settings.input_tag_value && tag1 == m_settings.embedding_tag_value)
            return true;
    } else {
        // need EdgeMesh
        // if () return false;
    }

    return false;
    // ask if this function could be commented out!
    // return TupleOperation::before();
}
bool EdgeSplitWithTag::execute()
{
    {
        EdgeSplit split_op(mesh(), input_tuple(), m_settings.split_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    Eigen::Vector3d p0 = m_pos_accessor.vector_attribute(input_tuple());
    Eigen::Vector3d p1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
    m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (p0 + p1);

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
