#include "TriMeshVertexSmoothOperation.hpp"
#include <wmtk/SimplicialComplex.hpp>

namespace wmtk {

TriMeshVertexSmoothOperation::TriMeshVertexSmoothOperation(
    Mesh& m,
    const Tuple& t,
    const Handles& handles)
    : Operation(m)
    , m_tuple(t)
    , m_pos_accessor(m.create_accessor(handles.position))
{}

std::string TriMeshVertexSmoothOperation::name() const
{
    return "vertex_smooth";
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
bool TriMeshVertexSmoothOperation::before() const
{
    return !m_mesh.is_outdated(m_tuple) && m_mesh.is_valid(m_tuple);
}

} // namespace wmtk
