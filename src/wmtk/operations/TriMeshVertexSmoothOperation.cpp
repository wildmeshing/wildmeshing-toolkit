#include "TriMeshVertexSmoothOperation.hpp"

#include <wmtk/SimplicialComplex.hpp>

namespace wmtk {
TriMeshVertexSmoothOperation::TriMeshVertexSmoothOperation(
    Mesh& m,
    const Tuple& t,
    const Settings& settings)
    : Operation(m)
    , m_tuple(t)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_smooth_boundary(settings.smooth_boundary)
{}

std::string TriMeshVertexSmoothOperation::name() const
{
    return "vertex_smooth";
}

bool TriMeshVertexSmoothOperation::before() const
{
    if (m_mesh.is_outdated(m_tuple) || !m_mesh.is_valid(m_tuple)) {
        return false;
    }
    if (!m_smooth_boundary && m_mesh.is_vertex_boundary(m_tuple)) {
        return false;
    }
    return !m_mesh.is_outdated(m_tuple) && m_mesh.is_valid(m_tuple);
}

bool TriMeshVertexSmoothOperation::execute()
{
    const std::vector<Simplex> one_ring = SimplicialComplex::vertex_one_ring(m_tuple, m_mesh);
    Eigen::Vector3d p_mid(0, 0, 0);
    for (const Simplex& s : one_ring) {
        p_mid += m_pos_accessor.vector_attribute(s.tuple());
    }
    p_mid /= one_ring.size();

    m_pos_accessor.vector_attribute(m_tuple) = p_mid;

    return true;
}


} // namespace wmtk
