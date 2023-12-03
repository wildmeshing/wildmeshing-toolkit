#include "TaubinSmoothingWithinScalffold.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tet_mesh/VertexMoveToTargetPos.hpp>
#include <wmtk/operations/tri_mesh/VertexMoveToTargetPos.hpp>

namespace wmtk::components::internal {

TaubinSmoothingWithinScalffold::TaubinSmoothingWithinScalffold(
    MeshAttributeHandle<double>& position_handle,
    MeshAttributeHandle<double>& laplacian_vector_handle,
    MeshAttributeHandle<long>& vertex_tag_handle,
    MeshAttributeHandle<long>& vertex_todo_handle,
    const long input_vertex_tag_value,
    const long scalffold_vertex_tag_value,
    const double shrink_alpha,
    const double inflate_alpha)
    : m_position_handle(position_handle)
    , m_laplacian_vector_handle(laplacian_vector_handle)
    , m_vertex_tag_handle(vertex_tag_handle)
    , m_vertex_todo_handle(vertex_todo_handle)
    , m_input_vertex_tag_value(input_vertex_tag_value)
    , m_scalffold_vertex_tag_value(scalffold_vertex_tag_value)
    , m_shrink_alpha(shrink_alpha)
    , m_inflate_alpha(inflate_alpha)
{}

void TaubinSmoothingWithinScalffold::process(TriMesh& m_mesh, const long iterations)
{
    using namespace operations;
    Scheduler scheduler(m_mesh);
    // setup shrink operation
    {
        OperationSettings<tri_mesh::VertexMoveToTargetPos> settings;
        settings.alpha = m_shrink_alpha;
        settings.move_vector = m_laplacian_vector_handle;
        settings.position = m_position_handle;
        settings.todo_tag_handle = m_vertex_todo_handle;
        settings.initialize_invariants(m_mesh);
        scheduler.add_operation_type<tri_mesh::VertexMoveToTargetPos>(
            "taubin_smoothing_shrink",
            settings);
    }
    // setup inflate operation
    {
        OperationSettings<tri_mesh::VertexMoveToTargetPos> settings;
        settings.alpha = m_inflate_alpha;
        settings.move_vector = m_laplacian_vector_handle;
        settings.position = m_position_handle;
        settings.todo_tag_handle = m_vertex_todo_handle;
        settings.initialize_invariants(m_mesh);
        scheduler.add_operation_type<tri_mesh::VertexMoveToTargetPos>(
            "taubin_smoothing_inflate",
            settings);
    }

    // smoothing
    Accessor<double> acc_pos = m_mesh.create_accessor(m_position_handle);
    Accessor<double> acc_laplacian = m_mesh.create_accessor(m_laplacian_vector_handle);
    Accessor<long> acc_vertex_tag = m_mesh.create_accessor(m_vertex_tag_handle);
    Accessor<long> acc_todo = m_mesh.create_accessor(m_vertex_todo_handle);
    for (int i = 0; i < iterations; ++i) {
        // prepare for the tag and laplacian vector
        {
            for (const Tuple& t : m_mesh.get_all(PrimitiveType::Vertex)) {
                if (acc_vertex_tag.scalar_attribute(t) == m_input_vertex_tag_value) {
                    acc_todo.scalar_attribute(t) = 1;
                    // compute the laplacian vector
                    Eigen::Vector3d p = acc_pos.vector_attribute(t);
                    Eigen::Vector3d laplacian = Eigen::Vector3d::Zero();
                    double times = 0.0;
                    for (const Simplex& s : SimplicialComplex::vertex_one_ring(m_mesh, t)) {
                        if (acc_vertex_tag.scalar_attribute(s.tuple()) ==
                            m_input_vertex_tag_value) {
                            Eigen::Vector3d t_p = acc_pos.vector_attribute(s.tuple());
                            laplacian += t_p - p;
                            times++;
                        }
                    }
                    if (times < 2) {
                        throw std::runtime_error(
                            "taubin smoothing's input vertex is isolated! -- neighbour < 2");
                    }
                    acc_laplacian.vector_attribute(t) = laplacian / times;
                } else {
                    acc_todo.scalar_attribute(t) = 0;
                    acc_laplacian.vector_attribute(t) = Eigen::Vector3d::Zero();
                }
            }
        }
        while (true) {
            scheduler.run_operation_on_all(PrimitiveType::Vertex, "taubin_smoothing_shrink");
            if (scheduler.number_of_successful_operations() == 0) {
                break;
            }
        }
        // prepare for the tag
        {
            for (const Tuple& t : m_mesh.get_all(PrimitiveType::Vertex)) {
                if (acc_vertex_tag.scalar_attribute(t) == m_input_vertex_tag_value) {
                    acc_todo.scalar_attribute(t) = 1;
                } else {
                    acc_todo.scalar_attribute(t) = 0;
                }
            }
        }
        while (true) {
            scheduler.run_operation_on_all(PrimitiveType::Vertex, "taubin_smoothing_inflate");
            if (scheduler.number_of_successful_operations() == 0) {
                break;
            }
        }
    }
}
void TaubinSmoothingWithinScalffold::process(TetMesh& m_mesh, const long iterations)
{
    Scheduler scheduler(m_mesh);
    // setup shrink operation
    {}
    // setup inflate operation
    {}
    // smoothing
    {}
}

} // namespace wmtk::components::internal
