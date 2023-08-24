#include "VertexSmooth.hpp"

#include <wmtk/SimplicialComplex.hpp>

namespace wmtk::operations::tri_mesh {
VertexSmooth::VertexSmooth(Mesh& m, const Tuple& t, const OperationSettings<VertexSmooth>& settings)
    : Operation(m)
    , m_input_tuple(t)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_settings{settings}
{}

std::string VertexSmooth::name() const
{
    return "tri_mesh_vertex_smooth";
}

const Tuple& VertexSmooth::return_tuple() const
{
    return m_output_tuple;
}

bool VertexSmooth::before() const
{
    if (mesh().is_outdated(m_input_tuple) || !mesh().is_valid(m_input_tuple)) {
        return false;
    }
    if (!m_settings.smooth_boundary && mesh().is_boundary_vertex(m_input_tuple)) {
        return false;
    }
    return true;
}

bool VertexSmooth::execute()
{
    const std::vector<Simplex> one_ring = SimplicialComplex::vertex_one_ring(mesh(), m_input_tuple);
    auto p_mid = m_pos_accessor.vector_attribute(m_input_tuple);
    p_mid = Eigen::Vector3d::Zero();
    for (const Simplex& s : one_ring) {
        p_mid += m_pos_accessor.vector_attribute(s.tuple());
    }
    p_mid /= one_ring.size();

    const SimplicialComplex star =
        SimplicialComplex::closed_star(mesh(), Simplex::vertex(m_input_tuple));
    const auto star_faces = star.get_faces();
    std::vector<Tuple> incident_face_tuple;
    incident_face_tuple.reserve(star_faces.size());
    for (const Simplex& s : star_faces) {
        incident_face_tuple.emplace_back(s.tuple());
    }


    std::vector<Tuple> tuple_updates{m_input_tuple};
    update_cell_hash(incident_face_tuple, tuple_updates);

    assert(mesh().is_outdated(m_input_tuple));

    m_output_tuple = tuple_updates[0];
    assert(!mesh().is_outdated(m_output_tuple));


    return true;
}


} // namespace wmtk::operations::tri_mesh
