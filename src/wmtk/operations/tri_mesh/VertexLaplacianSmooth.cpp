#include "VertexLaplacianSmooth.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::operations::tri_mesh {
VertexLaplacianSmooth::VertexLaplacianSmooth(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexLaplacianSmooth>& settings)
    : VertexAttributesUpdateBase(m, t, settings.base_settings)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_settings{settings}
{}

std::string VertexLaplacianSmooth::name() const
{
    return "tri_mesh_vertex_laplacian_smooth";
}

const Tuple& VertexLaplacianSmooth::return_tuple() const
{
    return m_output_tuple;
}

bool VertexLaplacianSmooth::before() const
{
    if (!mesh().is_valid_slow(input_tuple())) {
        return false;
    }
    if (!m_settings.smooth_boundary && mesh().is_boundary_vertex(input_tuple())) {
        return false;
    }
    return true;
}

bool VertexLaplacianSmooth::execute()
{
    const std::vector<Simplex> one_ring = SimplicialComplex::vertex_one_ring(mesh(), input_tuple());
    auto p_mid = m_pos_accessor.vector_attribute(input_tuple());
    p_mid = Eigen::Vector3d::Zero();
    for (const Simplex& s : one_ring) {
        p_mid += m_pos_accessor.vector_attribute(s.tuple());
    }
    p_mid /= one_ring.size();

    const SimplicialComplex star =
        SimplicialComplex::closed_star(mesh(), Simplex::vertex(input_tuple()));
    const auto star_faces = star.get_faces();
    std::vector<Tuple> incident_face_tuple;
    incident_face_tuple.reserve(star_faces.size());
    for (const Simplex& s : star_faces) {
        incident_face_tuple.emplace_back(s.tuple());
    }

    update_cell_hashes(incident_face_tuple);

    assert(!mesh().is_valid_slow(input_tuple()));

    m_output_tuple = resurrect_tuple(input_tuple());
    assert(mesh().is_valid_slow(m_output_tuple));


    return true;
}


} // namespace wmtk::operations::tri_mesh
