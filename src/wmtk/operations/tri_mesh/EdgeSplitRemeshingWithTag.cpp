#include "EdgeSplitRemeshingWithTag.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/SplitScallfoldInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {


void OperationSettings<tri_mesh::EdgeSplitRemeshingWithTag>::initialize_invariants(const TriMesh& m)
{
    split_settings.initialize_invariants(m);
    split_settings.invariants.add(std::make_shared<SplitScallfoldInvariant>(
        m,
        position,
        vertex_tag,
        min_squared_length,
        input_tag_value,
        embedding_tag_value,
        offset_tag_value));
}

bool OperationSettings<tri_mesh::EdgeSplitRemeshingWithTag>::are_invariants_initialized() const
{
    return split_settings.are_invariants_initialized() &&
           find_invariants_in_collection_by_type<SplitScallfoldInvariant>(
               split_settings.invariants);
}
namespace tri_mesh {
EdgeSplitRemeshingWithTag::EdgeSplitRemeshingWithTag(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeSplitRemeshingWithTag>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.split_settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_vertex_tag_accessor{m.create_accessor(settings.vertex_tag)}
    , m_edge_tag_accessor{m.create_accessor(settings.edge_tag)}
    , m_settings{settings}
{}
std::string EdgeSplitRemeshingWithTag::name() const
{
    return "edge_split_remeshing_with_tag";
}
Tuple EdgeSplitRemeshingWithTag::return_tuple() const
{
    return m_output_tuple;
}
bool EdgeSplitRemeshingWithTag::before() const
{
    return TupleOperation::before();
}
bool EdgeSplitRemeshingWithTag::execute()
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
    // v tag depends on its neighbour
    // ...

    // e tag, use star
    // ...

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
