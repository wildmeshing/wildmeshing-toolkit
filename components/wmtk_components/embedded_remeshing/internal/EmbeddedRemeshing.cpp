#include "EmbeddedRemeshing.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tet_mesh/VertexLaplacianSmoothWithTags.hpp>
#include <wmtk/operations/tet_mesh/VertexPushOffset.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseToMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwapValence.hpp>
#include <wmtk/operations/tri_mesh/VertexLaplacianSmoothWithTags.hpp>
#include <wmtk/operations/tri_mesh/VertexPushOffset.hpp>

namespace wmtk::components::internal {

EmbeddedRemeshing::EmbeddedRemeshing(
    MeshAttributeHandle<long>& vertex_tag_handle,
    MeshAttributeHandle<long>& edge_tag_handle,
    MeshAttributeHandle<long>& face_tag_handle,
    MeshAttributeHandle<long>& todo_vertex_handle,
    MeshAttributeHandle<double>& pos_handle,
    const long input_tag_value,
    const long embedding_tag_value,
    const long offset_tag_value,
    const double inflate_len,
    const double length,
    const bool lock_boundary)
    : m_vertex_tag_handle(vertex_tag_handle)
    , m_edge_tag_handle(edge_tag_handle)
    , m_face_tag_handle(face_tag_handle)
    , m_todo_vertex_handle(todo_vertex_handle)
    , m_pos_handle(pos_handle)
    , m_input_tag_value(input_tag_value)
    , m_embedding_tag_value(embedding_tag_value)
    , m_offset_tag_value(offset_tag_value)
    , m_inflate_len(inflate_len)
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
{}

void EmbeddedRemeshing::remeshing(TriMesh& mesh, const long iterations)
{
    Scheduler scheduler(mesh);

    {
        operations::OperationSettings<wmtk::operations::tri_mesh::VertexLaplacianSmoothWithTags>
            setting;
        setting.edge_tag_handle = m_edge_tag_handle;
        setting.position = m_pos_handle;
        setting.todo_tag_handle = m_todo_vertex_handle;
        setting.vertex_tag_handle = m_vertex_tag_handle;
        setting.embedding_tag_value = m_embedding_tag_value;
        setting.offset_tag_value = m_offset_tag_value;
        setting.initialize_invariants(mesh);
        scheduler.add_operation_type<operations::tri_mesh::VertexLaplacianSmoothWithTags>(
            "vertex_relocation",
            setting);
    }

    {
        operations::OperationSettings<operations::tri_mesh::VertexPushOffset> settings;
        settings.edge_tag_handle = m_edge_tag_handle;
        settings.embedding_tag_value = m_embedding_tag_value;
        settings.offset_tag_value = m_offset_tag_value;
        settings.input_tag_value = m_input_tag_value;
        settings.offset_len = m_inflate_len;
        settings.position = m_pos_handle;
        settings.vertex_tag_handle = m_vertex_tag_handle;
        settings.todo_tag_handle = m_todo_vertex_handle;
        settings.initialize_invariants(mesh);

        scheduler.add_operation_type<operations::tri_mesh::VertexPushOffset>(
            "vertex_push",
            settings);
    }

    for (long i = 0; i < iterations; ++i) {
        // smoothing
        {
            Accessor<long> acc_vertex_tag = mesh.create_accessor(m_vertex_tag_handle);
            Accessor<long> acc_todo_tag = mesh.create_accessor(m_todo_vertex_handle);
            for (const Tuple& t : mesh.get_all(PrimitiveType::Vertex)) {
                if (acc_vertex_tag.scalar_attribute(t) != m_input_tag_value) {
                    acc_todo_tag.scalar_attribute(t) = 1;
                } else {
                    acc_todo_tag.scalar_attribute(t) = 0;
                }
            }

            while (true) {
                scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_relocation");
                if (scheduler.number_of_successful_operations() == 0) {
                    break;
                }
            }
        }
        // pushing
        {
            Accessor<long> acc_vertex_tag = mesh.create_accessor(m_vertex_tag_handle);
            Accessor<long> acc_todo_tag = mesh.create_accessor(m_todo_vertex_handle);
            for (const Tuple& t : mesh.get_all(PrimitiveType::Vertex)) {
                if (acc_vertex_tag.scalar_attribute(t) == m_offset_tag_value) {
                    acc_todo_tag.scalar_attribute(t) = 1;
                } else {
                    acc_todo_tag.scalar_attribute(t) = 0;
                }
            }

            while (true) {
                scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_push");
                if (scheduler.number_of_successful_operations() == 0) {
                    break;
                }
            }
        }
    }
}

void EmbeddedRemeshing::remeshing(TetMesh& mesh, const long iterations)
{
    Scheduler scheduler(mesh);
    {
        operations::OperationSettings<operations::tet_mesh::VertexLaplacianSmoothWithTags> settings;
        settings.edge_tag_handle = m_edge_tag_handle;
        settings.embedding_tag_value = m_embedding_tag_value;
        settings.offset_tag_value = m_offset_tag_value;
        settings.position = m_pos_handle;
        settings.todo_tag_handle = m_todo_vertex_handle;
        settings.vertex_tag_handle = m_vertex_tag_handle;
        settings.initialize_invariants(mesh);

        scheduler.add_operation_type<operations::tet_mesh::VertexLaplacianSmoothWithTags>(
            "vertex_relocation",
            settings);
    }

    {
        operations::OperationSettings<operations::tet_mesh::VertexPushOffset> settings;
        settings.edge_tag_handle = m_edge_tag_handle;
        settings.embedding_tag_value = m_embedding_tag_value;
        settings.offset_tag_value = m_offset_tag_value;
        settings.input_tag_value = m_input_tag_value;
        settings.offset_len = m_inflate_len;
        settings.position = m_pos_handle;
        settings.vertex_tag_handle = m_vertex_tag_handle;
        settings.todo_tag_handle = m_todo_vertex_handle;
        settings.face_tag_handle = m_face_tag_handle;
        settings.initialize_invariants(mesh);

        scheduler.add_operation_type<operations::tet_mesh::VertexPushOffset>(
            "vertex_push",
            settings);
    }
    for (long i = 0; i < iterations; ++i) {
        // smoothing
        {
            Accessor<long> acc_vertex_tag = mesh.create_accessor(m_vertex_tag_handle);
            Accessor<long> acc_todo_tag = mesh.create_accessor(m_todo_vertex_handle);
            for (const Tuple& t : mesh.get_all(PrimitiveType::Vertex)) {
                if (acc_vertex_tag.scalar_attribute(t) != m_input_tag_value) {
                    acc_todo_tag.scalar_attribute(t) = 1;
                } else {
                    acc_todo_tag.scalar_attribute(t) = 0;
                }
            }

            while (true) {
                scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_relocation");
                if (scheduler.number_of_successful_operations() == 0) {
                    break;
                }
            }
        }

        // pushing
        {
            Accessor<long> acc_vertex_tag = mesh.create_accessor(m_vertex_tag_handle);
            Accessor<long> acc_todo_tag = mesh.create_accessor(m_todo_vertex_handle);
            for (const Tuple& t : mesh.get_all(PrimitiveType::Vertex)) {
                if (acc_vertex_tag.scalar_attribute(t) == m_offset_tag_value) {
                    acc_todo_tag.scalar_attribute(t) = 1;
                } else {
                    acc_todo_tag.scalar_attribute(t) = 0;
                }
            }

            while (true) {
                scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_push");
                if (scheduler.number_of_successful_operations() == 0) {
                    break;
                }
            }
        }
    }
}

} // namespace wmtk::components::internal
