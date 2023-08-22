#include "TriMeshCollapseEdgeToMidpointOperation.hpp"

#include <wmtk/TriMesh.hpp>
#include "TriMeshCollapseEdgeOperation.hpp"

namespace wmtk::operations {
TriMeshCollapseEdgeToMidpointOperation::TriMeshCollapseEdgeToMidpointOperation(
    wmtk::Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshCollapseEdgeToMidpointOperation>& settings)
    : Operation(m)
    , m_input_tuple{t}
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{
    p0 = m_pos_accessor.vector_attribute(m_input_tuple);
    p1 = m_pos_accessor.vector_attribute(m_mesh.switch_vertex(m_input_tuple));
}

std::string TriMeshCollapseEdgeToMidpointOperation::name() const
{
    return "tri_mesh_collapse_edge_to_mid";
}

Tuple TriMeshCollapseEdgeToMidpointOperation::return_tuple() const
{
    return m_output_tuple;
}

bool TriMeshCollapseEdgeToMidpointOperation::before() const
{
    if (m_mesh.is_outdated(m_input_tuple) || !m_mesh.is_valid(m_input_tuple)) {
        return false;
    }

    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared < m_settings.max_squared_length;
}

bool TriMeshCollapseEdgeToMidpointOperation::execute()
{
    bool v0_is_boundary = false;
    bool v1_is_boundary = false;
    if (m_settings.collapse_towards_boundary) {
        v0_is_boundary = m_mesh.is_boundary_vertex(m_input_tuple);
        v1_is_boundary = m_mesh.is_boundary_vertex(m_mesh.switch_vertex(m_input_tuple));
    }

    // collapse
    {
        OperationSettings<TriMeshCollapseEdgeOperation> op_settings;
        op_settings.collapse_boundary_edges = m_settings.collapse_boundary_edges;

        TriMeshCollapseEdgeOperation split_op(m_mesh, m_input_tuple, op_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    if (v0_is_boundary && !v1_is_boundary) {
        m_pos_accessor.vector_attribute(m_output_tuple) = p0;
    } else if (v1_is_boundary && !v0_is_boundary) {
        m_pos_accessor.vector_attribute(m_output_tuple) = p1;
    } else {
        m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (p0 + p1);
    }

    return true;
}

} // namespace wmtk::operations
