#include "Mesh.hpp"
#include <numeric>
#include <queue>

#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Logger.hpp>

#include "Primitive.hpp"

namespace wmtk {
bool Mesh::has_embedding() const
{
    return m_embedding;
}

submesh::Embedding& Mesh::get_embedding() const
{
    assert(has_embedding());
    return *m_embedding;
}
} // namespace wmtk
