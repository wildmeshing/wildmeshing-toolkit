#include "MeshInvariant.hpp"
namespace wmtk {
MeshInvariant::MeshInvariant(const Mesh& mesh)
//: m_mesh(mesh)    // TODO HACK include this
{
    MeshInvariant::m_mesh = &mesh; // TODO HACK remove this!!!
}

const Mesh& MeshInvariant::mesh() const
{
    return *(MeshInvariant::m_mesh);
}
} // namespace wmtk
