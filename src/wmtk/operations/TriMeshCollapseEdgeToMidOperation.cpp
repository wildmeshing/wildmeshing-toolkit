#include "TriMeshCollapseEdgeToMidOperation.hpp"

#include <wmtk/TriMesh.hpp>
#include "TriMeshCollapseEdgeOperation.hpp"
#include "TriMeshCollapseEdgeToMidOperation.hpp"

namespace wmtk {
wmtk::TriMeshCollapseEdgeToMidOperation::TriMeshCollapseEdgeToMidOperation(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshCollapseEdgeToMidOperation>& settings)
    : Operation(m)
    , m_input_tuple{t}
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_max_squared_length{settings.max_squared_length}
{
    p0 = m_pos_accessor.vector_attribute(m_input_tuple);
    p1 = m_pos_accessor.vector_attribute(m_mesh.switch_vertex(m_input_tuple));
}

std::string TriMeshCollapseEdgeToMidOperation::name() const
{
    return "tri_mesh_collapse_edge_to_mid";
}

Tuple TriMeshCollapseEdgeToMidOperation::return_tuple() const
{
    return m_output_tuple;
}

bool TriMeshCollapseEdgeToMidOperation::before() const
{
    if (m_mesh.is_outdated(m_input_tuple) || !m_mesh.is_valid(m_input_tuple)) {
        return false;
    }

    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared < m_max_squared_length;
}

bool TriMeshCollapseEdgeToMidOperation::execute()
{
    {
        TriMeshCollapseEdgeOperation split_op(m_mesh, m_input_tuple);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (p0 + p1);

    return true;
}

} // namespace wmtk
