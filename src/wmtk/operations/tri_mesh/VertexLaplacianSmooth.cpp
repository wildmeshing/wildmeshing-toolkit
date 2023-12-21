#include "VertexLaplacianSmooth.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>

namespace wmtk::operations {

OperationSettings<tri_mesh::VertexLaplacianSmooth>::OperationSettings(TriMesh& m)
    : OperationSettings<AttributesUpdateBase>(m)
{}

void OperationSettings<tri_mesh::VertexLaplacianSmooth>::create_invariants()
{
    OperationSettings<AttributesUpdateBase>::create_invariants();

    if (!smooth_boundary) {
        invariants->add(std::make_unique<InteriorVertexInvariant>(m_mesh));
    }
} // namespace wmtk::operations

namespace tri_mesh {
VertexLaplacianSmooth::VertexLaplacianSmooth(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<VertexLaplacianSmooth>& settings)
    : AttributesUpdateBase(m, t, settings)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Vertex);
}

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

    return AttributesUpdateBase::execute();
}

} // namespace tri_mesh
} // namespace wmtk::operations
