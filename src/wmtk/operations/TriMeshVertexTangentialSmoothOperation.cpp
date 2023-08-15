#include "TriMeshVertexTangentialSmoothOperation.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "TriMeshVertexSmoothOperation.hpp"

namespace wmtk {
TriMeshVertexTangentialSmoothOperation::TriMeshVertexTangentialSmoothOperation(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshVertexTangentialSmoothOperation>& settings)
    : Operation(m)
    , m_tuple{t}
    , m_pos_accessor{m.create_accessor<double>(settings.position)}
    , m_settings{settings}
{}

std::string TriMeshVertexTangentialSmoothOperation::name() const
{
    return "vertex_tangential_smooth";
}

bool TriMeshVertexTangentialSmoothOperation::before() const
{
    if (m_mesh.is_outdated(m_tuple) || !m_mesh.is_valid(m_tuple)) {
        return false;
    }
    if (!m_settings.smooth_boundary && m_mesh.is_boundary_vertex(m_tuple)) {
        return false;
    }
    return true;
}

bool TriMeshVertexTangentialSmoothOperation::execute()
{
    const Eigen::Vector3d p = m_pos_accessor.vector_attribute(m_tuple);
    {
        OperationSettings<TriMeshVertexSmoothOperation> op_settings;
        op_settings.position = m_pos_accessor.handle();
        TriMeshVertexSmoothOperation split_op(m_mesh, m_tuple, op_settings);
        if (!split_op()) {
            return false;
        }
    }
    const Eigen::Vector3d g = m_pos_accessor.vector_attribute(m_tuple); // center of gravity

    const Eigen::Vector3d n = mesh_utils::compute_vertex_normal(m_mesh, m_pos_accessor, m_tuple);

    if (n.squaredNorm() < 1e-10) {
        return false;
    }

    // following Botsch&Kobbelt - Remeshing for Multiresolution Modeling
    m_pos_accessor.vector_attribute(m_tuple) =
        p + m_settings.damping_factor * (Eigen::Matrix3d::Identity() - n * n.transpose()) * (g - p);

    return true;
}


} // namespace wmtk
