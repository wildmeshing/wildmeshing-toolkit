#include "TriMeshVertexTangentialSmoothOperation.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include "TriMeshVertexSmoothOperation.hpp"

namespace wmtk {
TriMeshVertexTangentialSmoothOperation::TriMeshVertexTangentialSmoothOperation(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshVertexTangentialSmoothOperation>& settings)
    : Operation(m)
    , m_tuple{t}
    , m_pos_accessor{m.create_accessor<double>(settings.position)}
    , m_smooth_boundary{settings.smooth_boundary}
    , m_damping_factor{settings.damping_factor}
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
    if (!m_smooth_boundary && m_mesh.is_boundary_vertex(m_tuple)) {
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

    // get normal of each face
    auto normal_area_weighted = [this](const std::vector<Tuple>& verts) {
        const Eigen::Vector3d p0 = m_pos_accessor.vector_attribute(verts[0]);
        const Eigen::Vector3d p1 = m_pos_accessor.vector_attribute(verts[1]);
        const Eigen::Vector3d p2 = m_pos_accessor.vector_attribute(verts[2]);
        return ((p0 - p2).cross(p1 - p2));
    };

    SimplicialComplex closed_star =
        SimplicialComplex::closed_star(m_mesh, Simplex::vertex(m_tuple));

    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    for (const Simplex& f : closed_star.get_faces()) {
        Tuple t = f.tuple();
        if (!m_mesh.is_ccw(t)) {
            t = m_mesh.switch_vertex(t);
        }
        const Tuple v0 = t;
        const Tuple v1 = m_mesh.switch_vertex(t);
        const Tuple v2 = m_mesh.switch_vertex(m_mesh.switch_edge(t));

        std::vector<Tuple> verts = {v0, v1, v2};
        n += normal_area_weighted(verts);
    }
    n.normalize();
    if (n.squaredNorm() < 1e-10) {
        return false;
    }

    // following Botsch&Kobbelt - Remeshing for Multiresolution Modeling
    m_pos_accessor.vector_attribute(m_tuple) =
        p + m_damping_factor * (Eigen::Matrix3d::Identity() - n * n.transpose()) * (g - p);

    return true;
}


} // namespace wmtk
