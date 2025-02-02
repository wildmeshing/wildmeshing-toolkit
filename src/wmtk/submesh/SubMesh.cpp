#include "SubMesh.hpp"

#include "Embedding.hpp"

namespace wmtk::submesh {

SubMesh::SubMesh(Embedding& embedding, int64_t submesh_id)
    : m_embedding(embedding)
    , m_submesh_id(submesh_id)
{}

void SubMesh::add_simplex(const Tuple& tuple, PrimitiveType pt)
{
    auto acc = m_embedding.tag_accessor(pt);
    acc.scalar_attribute(tuple) |= (int64_t)1 << m_submesh_id;
}

} // namespace wmtk::submesh
