#include "MeshInvariant.hpp"
namespace wmtk {
MeshInvariant::MeshInvariant(const Mesh& mesh)
    : m_mesh(mesh)
{}

const Mesh& MeshInvariant::mesh() const
{
    return m_mesh;
}
} // namespace wmtk
