#include "VertexLaplacianSmooth.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/link.hpp>


namespace wmtk::operations {

VertexLaplacianSmooth::VertexLaplacianSmooth(const attribute::MeshAttributeHandle& handle)
    : m_attibute_handle(handle)
{}


bool VertexLaplacianSmooth::operator()(Mesh& mesh, const simplex::Simplex& simplex)
{
    const std::vector<simplex::Simplex> one_ring =
        simplex::link(mesh, simplex).simplex_vector(PrimitiveType::Vertex);

    auto accessor = mesh.create_accessor<double>(m_attibute_handle);

    auto p_mid = accessor.vector_attribute(simplex);
    p_mid.setZero();
    for (const simplex::Simplex& s : one_ring) {
        p_mid = p_mid + accessor.vector_attribute(s);
    }
    p_mid = p_mid / one_ring.size();

    return true;
}

} // namespace wmtk::operations
