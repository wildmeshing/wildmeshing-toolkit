#include "VertexLaplacianSmooth.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/link.hpp>


namespace wmtk::operations {

VertexLaplacianSmooth::VertexLaplacianSmooth(
    Mesh& m,
    const attribute::TypedAttributeHandle<double>& handle)
    : AttributesUpdateBase(m)
    , m_attibute_handle(handle)
{}

VertexLaplacianSmooth::VertexLaplacianSmooth(attribute::MeshAttributeHandle& handle)
    : VertexLaplacianSmooth(handle.mesh(), handle.as<double>())
{
    assert(handle.holds<double>());
}

std::vector<simplex::Simplex> VertexLaplacianSmooth::execute(const simplex::Simplex& simplex)
{
    auto accessor = mesh().create_accessor<double>(m_attibute_handle);
    const std::vector<simplex::Simplex> one_ring =
        simplex::link(mesh(), simplex).simplex_vector(PrimitiveType::Vertex);
    auto p_mid = accessor.vector_attribute(simplex.tuple());
    p_mid = Eigen::Vector3d::Zero();
    for (const simplex::Simplex& s : one_ring) {
        p_mid += accessor.vector_attribute(s.tuple());
    }
    p_mid /= one_ring.size();

    return AttributesUpdateBase::execute(simplex);
}

} // namespace wmtk::operations
