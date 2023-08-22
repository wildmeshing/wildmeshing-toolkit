#include "VertexSmooth.hpp"

#include <wmtk/SimplicialComplex.hpp>

namespace wmtk::operations::tri_mesh {
VertexSmooth::VertexSmooth(
    wmtk::Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmooth>& settings)
    : Operation(m)
    , m_tuple(t)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_settings{settings}
{}

std::string VertexSmooth::name() const
{
    return "tri_mesh_vertex_smooth";
}

bool VertexSmooth::before() const
{
    if (m_mesh.is_outdated(m_tuple) || !m_mesh.is_valid(m_tuple)) {
        return false;
    }
    if (!m_settings.smooth_boundary && m_mesh.is_boundary_vertex(m_tuple)) {
        return false;
    }
    return true;
}

bool VertexSmooth::execute()
{
    const std::vector<Simplex> one_ring =
        SimplicialComplex::vertex_one_ring(dynamic_cast<TriMesh&>(m_mesh), m_tuple);
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


} // namespace wmtk::operations::tri_mesh
