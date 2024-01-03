#include "VertexLaplacianSmooth.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/link.hpp>


namespace wmtk::operations {

VertexLaplacianSmooth::VertexLaplacianSmooth(const MeshAttributeHandle<double>& handle)
    : m_attibute_handle(handle)
{}


bool VertexLaplacianSmooth::operator()(Mesh& mesh, const simplex::Simplex& simplex)
{
    auto accessor = mesh.create_accessor<double>(m_attibute_handle);
    const std::vector<simplex::Simplex> one_ring =
        simplex::link(mesh, simplex).simplex_vector(PrimitiveType::Vertex);
    auto p_mid = accessor.vector_attribute(simplex.tuple());
    p_mid.setZero();
    for (const simplex::Simplex& s : one_ring) {
        p_mid += accessor.vector_attribute(s.tuple());
    }
    p_mid /= one_ring.size();

    return true;
}

} // namespace wmtk::operations
