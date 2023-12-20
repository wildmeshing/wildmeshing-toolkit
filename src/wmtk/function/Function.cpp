#include "Function.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::function {

Function::Function(Mesh& mesh, const MeshAttributeHandle<double>& handle)
    : m_mesh(mesh)
{}

long Function::embedded_dimension() const
{
    return mesh().get_attribute_dimension(m_handle);
}


} // namespace wmtk::function
