#include "FaceSplitWithTag.hpp"
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>

namespace wmtk::operations {


void OperationSettings<tri_mesh::FaceSplitWithTag>::initialize_invariants(const TriMesh& m)
{
    face_split_settings.split_settings.initialize_invariants(m);
    face_split_settings.split_settings.invariants.add(
        std::make_shared<TodoInvariant>(m, split_todo));
}

bool OperationSettings<tri_mesh::FaceSplitWithTag>::are_invariants_initialized() const
{
    return face_split_settings.are_invariants_initialized() &&
           find_invariants_in_collection_by_type<TodoInvariant>(
               face_split_settings.split_settings.invariants);
}

namespace tri_mesh {
FaceSplitWithTag::FaceSplitWithTag(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<FaceSplitWithTag>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.face_split_settings.split_settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_vertex_tag_accessor{m.create_accessor(settings.vertex_tag)}
    , m_edge_tag_accessor{m.create_accessor(settings.edge_tag)}
    , m_split_todo_accessor{m.create_accessor(settings.split_todo)}
    , m_settings{settings}
{}
std::string FaceSplitWithTag::name() const
{
    return "tri_mesh_split_edge_at_midpoint";
}
Tuple FaceSplitWithTag::return_tuple() const
{
    return m_output_tuple;
}
bool FaceSplitWithTag::execute()
{
    const Eigen::Vector3d p0 = m_pos_accessor.vector_attribute(input_tuple());
    const Eigen::Vector3d p1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
    const Eigen::Vector3d p2 =
        m_pos_accessor.vector_attribute(mesh().switch_vertex(mesh().switch_edge(input_tuple())));
    {
        FaceSplitAtMidPoint split_op(mesh(), input_tuple(), m_settings.face_split_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    m_pos_accessor.vector_attribute(m_output_tuple) = (p0 + p1 + p2) / 3.0;
    m_vertex_tag_accessor.scalar_attribute(m_output_tuple) = m_settings.split_vertex_tag_value;
    m_split_todo_accessor.scalar_attribute(m_output_tuple) = 0;
    m_split_todo_accessor.scalar_attribute(mesh().switch_face(m_output_tuple)) = 0;
    m_split_todo_accessor.scalar_attribute(mesh().switch_face(mesh().switch_edge(m_output_tuple))) =
        0;

    if (m_settings.need_embedding_tag_value) {
        m_edge_tag_accessor.scalar_attribute(m_output_tuple) = m_settings.embedding_tag_value;
        m_edge_tag_accessor.scalar_attribute(mesh().switch_edge(m_output_tuple)) =
            m_settings.embedding_tag_value;
        m_edge_tag_accessor.scalar_attribute(mesh().switch_edge(
            mesh().switch_face(m_output_tuple))) = m_settings.embedding_tag_value;
    } else {
        m_edge_tag_accessor.scalar_attribute(m_output_tuple) =
            m_vertex_tag_accessor.const_scalar_attribute(mesh().switch_vertex(m_output_tuple));
        m_edge_tag_accessor.scalar_attribute(mesh().switch_edge(m_output_tuple)) =
            m_vertex_tag_accessor.const_scalar_attribute(
                mesh().switch_vertex(mesh().switch_edge(m_output_tuple)));
        m_edge_tag_accessor.scalar_attribute(
            mesh().switch_edge(mesh().switch_face(m_output_tuple))) =
            m_vertex_tag_accessor.const_scalar_attribute(
                mesh().switch_vertex(mesh().switch_edge(mesh().switch_face(m_output_tuple))));
    }

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
