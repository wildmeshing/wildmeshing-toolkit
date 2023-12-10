#include "RawSimplex.hpp"

#include "Simplex.hpp"
#include "faces_single_dimension.hpp"

namespace wmtk::simplex {
RawSimplex::RawSimplex(const Mesh& mesh, const std::vector<Tuple>& vertices)
{
    m_vertices.reserve(vertices.size());

    ConstAccessor<long> hash_accessor = mesh.get_const_cell_hash_accessor();

    for (size_t i = 0; i < vertices.size(); ++i) {
        m_vertices.emplace_back(
            mesh.is_valid(vertices[i], hash_accessor) ? mesh.id(vertices[i], PrimitiveType::Vertex)
                                                      : -1);
    }

    std::sort(m_vertices.begin(), m_vertices.end());
}

RawSimplex::RawSimplex(std::vector<long>&& vertices)
    : m_vertices{std::move(vertices)}
{
    std::sort(m_vertices.begin(), m_vertices.end());
}

RawSimplex::RawSimplex(const Mesh& mesh, const Simplex& simplex)
    : RawSimplex(mesh, faces_single_dimension_tuples(mesh, simplex, PrimitiveType::Vertex))
{}

long RawSimplex::dimension() const
{
    return m_vertices.size();
}

} // namespace wmtk::simplex