#include "IsotropicRemeshing.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/TriMeshCollapseEdgeToMidOperation.hpp>
#include <wmtk/operations/TriMeshSplitEdgeAtMidpointOperation.hpp>
#include <wmtk/operations/TriMeshSwapEdgeOperation.hpp>
#include <wmtk/operations/TriMeshVertexTangentialSmoothOperation.hpp>

namespace wmtk {
namespace components {
namespace internal {

IsotropicRemeshing::IsotropicRemeshing(TriMesh* mesh, const double length, const bool lock_boundary)
    : m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_position_handle{m_mesh->get_attribute_handle<double>("position", PrimitiveType::Vertex)}
    , m_scheduler(*m_mesh)
{
    if (!m_lock_boundary) {
        throw std::runtime_error("free boundary is not implemented yet");
    }

    // split
    {
        OperationSettings<TriMeshSplitEdgeAtMidpointOperation> split_settings;
        split_settings.position = m_position_handle;
        split_settings.min_squared_length = m_length_max * m_length_max;

        m_scheduler.add_operation_type<TriMeshSplitEdgeAtMidpointOperation>(
            "tri_mesh_split_edge_at_midpoint",
            split_settings);
    }
    // collapse
    {
        OperationSettings<TriMeshCollapseEdgeToMidOperation> op_settings;
        op_settings.position = m_position_handle;
        op_settings.max_squared_length = m_length_min * m_length_min;

        m_scheduler.add_operation_type<TriMeshCollapseEdgeToMidOperation>(
            "tri_mesh_collapse_edge_to_mid",
            op_settings);
    }
    // flip
    {
        OperationSettings<TriMeshSwapEdgeOperation> op_settings;
        op_settings.must_improve_valence = true;

        m_scheduler.add_operation_type<TriMeshSwapEdgeOperation>(
            "TriMeshSwapEdgeOperation",
            op_settings);
    }
    // smooth
    {
        OperationSettings<TriMeshVertexTangentialSmoothOperation> op_settings;
        op_settings.position = m_position_handle;

        m_scheduler.add_operation_type<TriMeshVertexTangentialSmoothOperation>(
            "vertex_tangential_smooth",
            op_settings);
    }
}

void IsotropicRemeshing::remeshing(const long iterations)
{
    for (long i = 0; i < iterations; ++i) {
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "tri_mesh_split_edge_at_midpoint");
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "tri_mesh_collapse_edge_to_mid");
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "TriMeshSwapEdgeOperation");
        m_scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_tangential_smooth");
    }
}

} // namespace internal
} // namespace components
} // namespace wmtk