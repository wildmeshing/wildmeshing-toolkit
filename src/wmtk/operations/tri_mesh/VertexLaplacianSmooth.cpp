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
    if (!tri_mesh::VertexAttributesUpdateBase::execute()) return false;
    Tuple tup = tri_mesh::VertexAttributesUpdateBase::return_tuple();
    const std::vector<Simplex> one_ring = SimplicialComplex::vertex_one_ring(mesh(), tup);
    auto p_mid = m_pos_accessor.vector_attribute(tup);
    p_mid = Eigen::Vector3d::Zero();
    for (const Simplex& s : one_ring) {
        p_mid += m_pos_accessor.vector_attribute(s.tuple());
    }
    p_mid /= one_ring.size();

    return true;
}


} // namespace wmtk::operations::tri_mesh
