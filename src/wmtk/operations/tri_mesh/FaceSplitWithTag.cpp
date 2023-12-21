#include "FaceSplitWithTag.hpp"
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>

namespace wmtk::operations {


void OperationSettings<tri_mesh::FaceSplitWithTag>::create_invariants()
{
    face_split_settings.create_invariants();

    invariants = std::make_shared<InvariantCollection>(m_mesh);

    invariants->add(std::make_shared<TodoInvariant>(m_mesh, split_todo));
}

namespace tri_mesh {
FaceSplitWithTag::FaceSplitWithTag(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<FaceSplitWithTag>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_vertex_tag_accessor{m.create_accessor(settings.vertex_tag)}
    , m_edge_tag_accessor{m.create_accessor(settings.edge_tag)}
    , m_split_todo_accessor{m.create_accessor(settings.split_todo)}
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Face);
}
std::string FaceSplitWithTag::name() const
{
    return "tri_mesh_split_face_with_tag";
}
Tuple FaceSplitWithTag::return_tuple() const
{
    return m_output_tuple;
}
bool FaceSplitWithTag::execute()
{
    // record ord tag for the three edges
    long t0, t1, t2;
    t0 = m_edge_tag_accessor.scalar_attribute(input_tuple());
    t1 = m_edge_tag_accessor.scalar_attribute(mesh().switch_edge(input_tuple()));
    t2 = m_edge_tag_accessor.scalar_attribute(
        mesh().switch_edge(mesh().switch_vertex(input_tuple())));

    std::optional<long> neighbor_face_todo;
    if (!mesh().is_boundary_edge(input_tuple())) {
        neighbor_face_todo =
            m_split_todo_accessor.scalar_attribute(mesh().switch_face(input_tuple()));
    }

    {
        FaceSplitAtMidPoint split_op(mesh(), input_simplex(), m_settings.face_split_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    m_vertex_tag_accessor.scalar_attribute(m_output_tuple) = m_settings.split_vertex_tag_value;

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

    // set three edges' tags back
    m_edge_tag_accessor.scalar_attribute(mesh().switch_edge(mesh().switch_vertex(m_output_tuple))) =
        t0;
    m_edge_tag_accessor.scalar_attribute(
        mesh().switch_edge(mesh().switch_vertex(mesh().switch_face(m_output_tuple)))) = t1;
    m_edge_tag_accessor.scalar_attribute(mesh().switch_edge(
        mesh().switch_vertex(mesh().switch_face(mesh().switch_edge(m_output_tuple))))) = t2;

    if (neighbor_face_todo.has_value()) {
        m_split_todo_accessor.scalar_attribute(mesh().switch_face(
            mesh().switch_edge(mesh().switch_vertex(m_output_tuple)))) = neighbor_face_todo.value();
    }

    return true;
}

std::vector<Simplex> FaceSplitWithTag::modified_primitives() const
{
    return {simplex::Simplex::vertex(m_output_tuple)};
}

std::vector<Simplex> FaceSplitWithTag::unmodified_primitives() const
{
    return {input_simplex()};
}
} // namespace tri_mesh
} // namespace wmtk::operations
