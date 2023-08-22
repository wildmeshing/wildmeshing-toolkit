#include "EdgeCollapseToMidpoint.hpp"

#include <wmtk/TriMesh.hpp>
#include "EdgeCollapse.hpp"

namespace wmtk::operations::tri_mesh {
EdgeCollapseToMidpoint::EdgeCollapseToMidpoint(
    wmtk::Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeCollapseToMidpoint>& settings)
    : Operation(m)
    , m_input_tuple{t}
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{
    p0 = m_pos_accessor.vector_attribute(m_input_tuple);
    p1 = m_pos_accessor.vector_attribute(m_mesh.switch_vertex(m_input_tuple));
}

std::string EdgeCollapseToMidpoint::name() const
{
    return "tri_mesh_edge_collapse_to_midpoint";
}

Tuple EdgeCollapseToMidpoint::return_tuple() const
{
    return m_output_tuple;
}

bool EdgeCollapseToMidpoint::before() const
{
    if (m_mesh.is_outdated(m_input_tuple) || !m_mesh.is_valid(m_input_tuple)) {
        return false;
    }

    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared < m_settings.max_squared_length;
}

bool EdgeCollapseToMidpoint::execute()
{
    bool v0_is_boundary = false;
    bool v1_is_boundary = false;
    if (m_settings.collapse_towards_boundary) {
        v0_is_boundary = m_mesh.is_boundary_vertex(m_input_tuple);
        v1_is_boundary = m_mesh.is_boundary_vertex(m_mesh.switch_vertex(m_input_tuple));
    }

    // collapse
    {
        OperationSettings<tri_mesh::EdgeCollapse> op_settings;
        op_settings.collapse_boundary_edges = m_settings.collapse_boundary_edges;

        tri_mesh::EdgeCollapse collapse_op(m_mesh, m_input_tuple, op_settings);
        if (!collapse_op()) {
            return false;
        }
        m_output_tuple = collapse_op.return_tuple();
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

} // namespace wmtk::operations::tri_mesh
