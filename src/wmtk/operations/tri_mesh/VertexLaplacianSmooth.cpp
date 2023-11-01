#include "VertexLaplacianSmooth.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>

namespace wmtk::operations {

void OperationSettings<tri_mesh::VertexLaplacianSmooth>::initialize_invariants(const TriMesh& m)
{
    base_settings.initialize_invariants(m);
    if (!smooth_boundary) {
        base_settings.invariants.add(std::make_unique<InteriorVertexInvariant>(m));
    }
} // namespace wmtk::operations

namespace tri_mesh {
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


bool VertexLaplacianSmooth::execute()
{
    const std::vector<Simplex> one_ring = SimplicialComplex::vertex_one_ring(mesh(), input_tuple());
    auto p_mid = m_pos_accessor.vector_attribute(input_tuple());
    p_mid = Eigen::Vector3d::Zero();
    for (const Simplex& s : one_ring) {
        p_mid += m_pos_accessor.vector_attribute(s.tuple());
    }
    p_mid /= one_ring.size();

    return tri_mesh::VertexAttributesUpdateBase::execute();
}

} // namespace tri_mesh
} // namespace wmtk::operations
