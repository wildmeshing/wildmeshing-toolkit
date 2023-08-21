#include "TriMeshVertexSmoothOperation.hpp"

#include <wmtk/SimplicialComplex.hpp>

namespace wmtk {
TriMeshVertexSmoothOperation::TriMeshVertexSmoothOperation(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshVertexSmoothOperation>& settings)
    : Operation(m)
    , m_tuple(t)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_settings{settings}
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
    if (!m_settings.smooth_boundary && m_mesh.is_boundary_vertex(m_tuple)) {
        return false;
    }
    return true;
}

bool TriMeshVertexSmoothOperation::execute()
{
    const std::vector<Simplex> one_ring = SimplicialComplex::vertex_one_ring(m_mesh, m_tuple);
    auto p_mid = m_pos_accessor.vector_attribute(m_tuple);
    p_mid = Eigen::Vector3d::Zero();
    for (const Simplex& s : one_ring) {
        p_mid += m_pos_accessor.vector_attribute(s.tuple());
    }
    p_mid /= one_ring.size();

    const SimplicialComplex star = SimplicialComplex::closed_star(m_mesh, Simplex::vertex(m_tuple));
    const auto star_faces = star.get_faces();
    std::vector<Tuple> incident_face_tuple;
    incident_face_tuple.reserve(star_faces.size());
    for (const Simplex& s : star_faces) {
        incident_face_tuple.emplace_back(s.tuple());
    }
    update_cell_hash(incident_face_tuple);


    return true;
}


} // namespace wmtk
