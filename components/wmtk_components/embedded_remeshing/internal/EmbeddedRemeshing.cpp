#include "EmbeddedRemeshing.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tet_mesh/VertexLaplacianSmoothWithTags.hpp>
#include <wmtk/operations/tet_mesh/VertexPushOffset.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseToMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitWithTag.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwapSafe.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwapValence.hpp>
#include <wmtk/operations/tri_mesh/VertexLaplacianSmoothWithTags.hpp>
#include <wmtk/operations/tri_mesh/VertexPushOffset.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>

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

void EmbeddedRemeshing::tri_split_offset_and_scalffold(TriMesh& mesh)
{
    Accessor<long> acc_vertex_tag = mesh.create_accessor<long>(m_vertex_tag_handle);
    Accessor<long> acc_edge_tag = mesh.create_accessor<long>(m_edge_tag_handle);

    operations::OperationSettings<wmtk::operations::tri_mesh::EdgeSplitAtMidpoint> setting;
    setting.split_settings.split_boundary_edges = false;
    setting.min_squared_length = m_length_max;
    setting.position = m_pos_handle;
    setting.initialize_invariants(mesh);

    for (const Tuple& t : mesh.get_all(PrimitiveType::Edge)) {
        if (mesh.is_boundary_edge(t)) {
            continue;
        }
        long et = acc_edge_tag.scalar_attribute(t);
        if (et == m_offset_tag_value) {
            // wmtk::operations::tri_mesh::EdgeSplitAtMidpoint op(mesh, t, setting);
            // if (op()) {
            //     const Tuple& ret = op.return_tuple();
            //     acc_vertex_tag.scalar_attribute(ret) = m_offset_tag_value;
            //     acc_edge_tag.scalar_attribute(ret) = m_offset_tag_value;
            //     acc_edge_tag.scalar_attribute(mesh.switch_edge(
            //         mesh.switch_face(mesh.switch_edge(ret)))) = m_offset_tag_value;
            // }
        } else {
            long vt0 = acc_vertex_tag.scalar_attribute(t);
            long vt1 = acc_vertex_tag.scalar_attribute(mesh.switch_vertex(t));
            if (vt0 == m_embedding_tag_value && vt1 == m_embedding_tag_value) {
                wmtk::operations::tri_mesh::EdgeSplitAtMidpoint op(mesh, t, setting);
                if (op()) {
                    const Tuple& ret = op.return_tuple();
                    acc_vertex_tag.scalar_attribute(ret) = m_embedding_tag_value;
                    acc_edge_tag.scalar_attribute(ret) = m_embedding_tag_value;
                    acc_edge_tag.scalar_attribute(mesh.switch_edge(
                        mesh.switch_face(mesh.switch_edge(ret)))) = m_embedding_tag_value;
                }
            }
        }
    }
}

void EmbeddedRemeshing::tri_collapse_scalffold(TriMesh& mesh)
{
    Accessor<long> acc_vertex_tag = mesh.create_accessor<long>(m_vertex_tag_handle);

    operations::OperationSettings<wmtk::operations::tri_mesh::EdgeCollapseToMidpoint> setting;
    setting.collapse_settings.collapse_boundary_edges = false;
    setting.collapse_settings.collapse_boundary_vertex_to_interior = false;
    setting.collapse_towards_boundary = true;
    setting.max_squared_length = m_length_min;
    setting.position = m_pos_handle;
    setting.initialize_invariants(mesh);

    for (const Tuple& t : mesh.get_all(PrimitiveType::Edge)) {
        long vt0 = acc_vertex_tag.scalar_attribute(t);
        long vt1 = acc_vertex_tag.scalar_attribute(mesh.switch_vertex(t));
        if (mesh.is_boundary_edge(t)) {
            continue;
        }
        if (vt0 == m_embedding_tag_value && vt1 == m_embedding_tag_value) {
            wmtk::operations::tri_mesh::EdgeCollapseToMidpoint op(mesh, t, setting);
            op();
        } else if (vt0 == m_embedding_tag_value && vt1 == m_offset_tag_value) {
            wmtk::operations::tri_mesh::EdgeCollapseToMidpoint op(mesh, t, setting);
            op();
        } else if (vt1 == m_embedding_tag_value && vt0 == m_offset_tag_value) {
            wmtk::operations::tri_mesh::EdgeCollapseToMidpoint op(
                mesh,
                mesh.switch_vertex(t),
                setting);
            op();
        }
    }
}

void EmbeddedRemeshing::tri_swap_scalffold(TriMesh& mesh)
{
    Accessor<long> acc_vertex_tag = mesh.create_accessor<long>(m_vertex_tag_handle);
    Accessor<long> acc_edge_tag = mesh.create_accessor<long>(m_edge_tag_handle);
    Accessor<double> acc_pos = mesh.create_accessor<double>(m_pos_handle);

    operations::OperationSettings<wmtk::operations::tri_mesh::EdgeSwapSafe> setting;
    setting.collapse_settings.collapse_towards_boundary = true;
    setting.collapse_settings.collapse_settings.collapse_boundary_edges = false;
    setting.collapse_settings.collapse_settings.collapse_boundary_vertex_to_interior = false;
    setting.collapse_settings.position = m_pos_handle;
    setting.split_settings.position = m_pos_handle;
    setting.initialize_invariants(mesh);

    for (const Tuple& t : mesh.get_all(PrimitiveType::Edge)) {
        if (mesh.is_boundary_edge(t)) {
            continue;
        }
        long vt0 = acc_vertex_tag.scalar_attribute(t);
        long vt1 = acc_vertex_tag.scalar_attribute(mesh.switch_vertex(t));
        long et = acc_edge_tag.scalar_attribute(t);
        if ((vt0 == m_embedding_tag_value && vt1 == m_embedding_tag_value) ||
            (vt0 == m_offset_tag_value && vt1 == m_offset_tag_value &&
             et == m_embedding_tag_value)) {
            const Tuple& f0 = t;
            const Tuple& f1 = mesh.switch_face(t);
            Eigen::Vector3d v0 = acc_pos.const_vector_attribute(f0);
            Eigen::Vector3d v1 = acc_pos.const_vector_attribute(mesh.switch_vertex(f0));
            Eigen::Vector3d v2 =
                acc_pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(f0)));
            Eigen::Vector3d v3 =
                acc_pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(f1)));

            const long val_before =
                (std::fabs((v2 - v0).cross((v1 - v0)).z()) +
                 std::fabs((v1 - v0).cross((v3 - v0)).z())) /
                ((v1 - v0).squaredNorm() * 2 + (v2 - v0).squaredNorm() + (v2 - v1).squaredNorm() +
                 (v3 - v0).squaredNorm() + (v3 - v1).squaredNorm());
            const long val_after =
                (std::fabs((v0 - v3).cross((v2 - v3)).z()) +
                 std::fabs((v1 - v3).cross((v2 - v3)).z())) /
                ((v2 - v3).squaredNorm() * 2 + (v2 - v0).squaredNorm() + (v2 - v1).squaredNorm() +
                 (v3 - v0).squaredNorm() + (v3 - v1).squaredNorm());

            if (val_after >= val_before) {
                wmtk::operations::tri_mesh::EdgeSwapSafe op(mesh, t, setting);
                op();
            }
        }
        // else if (
        //     (vt0 == m_offset_tag_value && vt1 == m_input_tag_value) ||
        //     (vt1 == m_offset_tag_value && vt0 == m_input_tag_value)) {
        //     const Tuple& f0 = t;
        //     const Tuple& f1 = mesh.switch_face(t);
        //     Eigen::Vector3d v0 = acc_pos.const_vector_attribute(f0);
        //     Eigen::Vector3d v1 = acc_pos.const_vector_attribute(mesh.switch_vertex(f0));
        //     Eigen::Vector3d v2 =
        //         acc_pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(f0)));
        //     Eigen::Vector3d v3 =
        //         acc_pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(f1)));
        //     if ((v0 - v1).squaredNorm() > (v2 - v3).squaredNorm()) {
        //         wmtk::operations::tri_mesh::EdgeSwapSafe op(mesh, t, setting);
        //         op();
        //     }
        // }
    }
}

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
        // split offset
        tri_split_offset_and_scalffold(mesh);

        // collapse scalffold
        tri_collapse_scalffold(mesh);

        // swap scalffold
        tri_swap_scalffold(mesh);

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
