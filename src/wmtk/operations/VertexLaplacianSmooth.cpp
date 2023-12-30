#include "VertexLaplacianSmooth.hpp"

#include <wmtk/Mesh.hpp>


namespace wmtk::operations {

VertexLaplacianSmooth::VertexLaplacianSmooth(Mesh& m, const MeshAttributeHandle<double>& handle)
    : AttributesUpdateBase(m)
    , m_attibute_handle(handle)
{}

std::vector<Simplex> VertexLaplacianSmooth::execute(const Simplex& simplex)
{
    auto accessor = mesh().create_accessor<double>(m_attibute_handle);
    const std::vector<Simplex> one_ring = simplex::vertex_one_ring(mesh(), simplex.tuple());
    auto p_mid = accessor.vector_attribute(simplex.tuple());
    p_mid = Eigen::Vector3d::Zero();
    for (const Simplex& s : one_ring) {
        p_mid += accessor.vector_attribute(s.tuple());
    }
    p_mid /= one_ring.size();

    return AttributesUpdateBase::execute(simplex);
}

} // namespace wmtk::operations
