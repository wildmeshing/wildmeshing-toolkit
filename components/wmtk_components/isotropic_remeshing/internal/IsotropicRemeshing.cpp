#include "IsotropicRemeshing.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/TriMeshCollapseEdgeToMidpointOperation.hpp>
#include <wmtk/operations/TriMeshSplitEdgeAtMidpointOperation.hpp>
#include <wmtk/operations/TriMeshSwapEdgeOperation.hpp>
#include <wmtk/operations/TriMeshVertexTangentialSmoothOperation.hpp>

namespace wmtk::components::internal {

IsotropicRemeshing::IsotropicRemeshing(TriMesh& mesh, const double length, const bool lock_boundary)
    : m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_position_handle{m_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex)}
    , m_scheduler(m_mesh)
{
    // split
    {
        OperationSettings<TriMeshSplitEdgeAtMidpointOperation> split_settings;
        split_settings.position = m_position_handle,
        split_settings.min_squared_length = m_length_min * m_length_min;
        split_settings.split_settings.split_boundary_edges = !m_lock_boundary;
        split_settings.initialize_invariants(m_mesh);

        m_scheduler.add_operation_type<TriMeshSplitEdgeAtMidpointOperation>(
            "split",
            split_settings);
    }
    // collapse
    {
        OperationSettings<TriMeshCollapseEdgeToMidpointOperation> op_settings;
        op_settings.position = m_position_handle,
        op_settings.max_squared_length = m_length_max * m_length_max;
        op_settings.collapse_settings.collapse_boundary_edges = !m_lock_boundary;
        op_settings.collapse_towards_boundary = true;

        op_settings.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<TriMeshCollapseEdgeToMidpointOperation>(
            "collapse",
            op_settings);
    }
    // flip
    {
        OperationSettings<TriMeshSwapEdgeOperation> op_settings{true};

        m_scheduler.add_operation_type<TriMeshSwapEdgeOperation>("swap", op_settings);
    }
    // smooth
    {
        OperationSettings<TriMeshVertexTangentialSmoothOperation> op_settings{
            m_position_handle,
            !m_lock_boundary};

        m_scheduler.add_operation_type<TriMeshVertexTangentialSmoothOperation>(
            "smooth",
            op_settings);
    }
}

void IsotropicRemeshing::remeshing(const long iterations)
{
    for (long i = 0; i < iterations; ++i) {
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "split");
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "collapse");
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "swap");
        m_scheduler.run_operation_on_all(PrimitiveType::Vertex, "smooth");
    }
}

} // namespace wmtk::components::internal
