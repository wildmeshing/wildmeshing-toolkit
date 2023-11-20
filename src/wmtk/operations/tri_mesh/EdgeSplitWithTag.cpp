#include "EdgeSplitWithTag.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {


void OperationSettings<tri_mesh::EdgeSplitWithTag>::initialize_invariants(const TriMesh& m)
{
    split_with_tag_settings.split_settings.initialize_invariants(m);
    split_with_tag_settings.split_settings.invariants.add(
        std::make_shared<TodoInvariant>(m, split_todo));
}

bool OperationSettings<tri_mesh::EdgeSplitWithTag>::are_invariants_initialized() const
{
    return split_with_tag_settings.split_settings.are_invariants_initialized() &&
           find_invariants_in_collection_by_type<TodoInvariant>(
               split_with_tag_settings.split_settings.invariants);
}
namespace tri_mesh {
EdgeSplitWithTag::EdgeSplitWithTag(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeSplitWithTag>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.split_with_tag_settings.split_settings.invariants, t)
    //, m_pos_accessor{m.create_accessor(settings.position)}
    , m_vertex_tag_accessor{m.create_accessor(settings.vertex_tag)}
    , m_edge_tag_accessor{m.create_accessor(settings.edge_tag)}
    , m_split_todo_accessor{m.create_accessor(settings.split_todo)}
    , m_settings{settings}
{}
std::string EdgeSplitWithTag::name() const
{
    return "tri_mesh_edge_split_with_tag";
}
Tuple EdgeSplitWithTag::return_tuple() const
{
    return m_output_tuple;
}
bool EdgeSplitWithTag::execute()
{
    // long et = m_edge_tag_accessor.scalar_attribute(input_tuple());
    std::optional<long> vt1;
    if (!mesh().is_boundary_edge(input_tuple())) {
        vt1 = m_vertex_tag_accessor.scalar_attribute(
            mesh().switch_vertex(mesh().switch_edge(mesh().switch_face(input_tuple()))));
    }

    {
        EdgeSplitAtMidpoint split_op(mesh(), input_tuple(), m_settings.split_with_tag_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }
    // m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (p0 + p1);
    m_split_todo_accessor.scalar_attribute(m_output_tuple) = 0;
    m_split_todo_accessor.scalar_attribute(
        mesh().switch_edge(mesh().switch_face(mesh().switch_edge(m_output_tuple)))) = 0;
    // two split edge should be the split edge value, and the split vertex should be the split
    // vertex value
    m_vertex_tag_accessor.scalar_attribute(m_output_tuple) = m_settings.split_vertex_tag_value;
    m_edge_tag_accessor.scalar_attribute(mesh().switch_edge(m_output_tuple)) =
        m_settings.split_edge_tag_value;
    if (!mesh().is_boundary_edge(m_output_tuple)) {
        m_edge_tag_accessor.scalar_attribute(mesh().switch_edge(
            mesh().switch_face(m_output_tuple))) = m_settings.split_edge_tag_value;
    }

    // if the embedding tag value is needed, then assign two edges connect to the original edges
    // with the embedding tag value, otherwise assign them with their neighbour's vertex's tag value
    if (m_settings.need_embedding_tag_value) {
        m_edge_tag_accessor.scalar_attribute(m_output_tuple) = m_settings.embedding_tag_value;
        if (!mesh().is_boundary_edge(mesh().switch_edge(m_output_tuple))) {
            m_edge_tag_accessor.scalar_attribute(
                mesh().switch_edge(mesh().switch_face(mesh().switch_edge(m_output_tuple)))) =
                m_settings.embedding_tag_value;
        }
    } else {
        m_edge_tag_accessor.scalar_attribute(m_output_tuple) =
            m_vertex_tag_accessor.scalar_attribute(mesh().switch_vertex(m_output_tuple));
        if (!mesh().is_boundary_edge(mesh().switch_edge(m_output_tuple))) {
            m_edge_tag_accessor.scalar_attribute(
                mesh().switch_edge(mesh().switch_face(mesh().switch_edge(m_output_tuple)))) =
                m_vertex_tag_accessor.scalar_attribute(mesh().switch_vertex(
                    mesh().switch_edge(mesh().switch_face(mesh().switch_edge(m_output_tuple)))));
        }
    }

    if (vt1.has_value()) {
        m_vertex_tag_accessor.scalar_attribute(mesh().switch_vertex(
            mesh().switch_edge(mesh().switch_face(m_output_tuple)))) = vt1.value();
    }

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations