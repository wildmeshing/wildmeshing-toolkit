#include "Function.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::function {

Function::Function(Mesh& mesh, const MeshAttributeHandle<double>& handle)
    : m_mesh(mesh)
    , m_handle(handle)
{}

int64_t Function::embedded_dimension() const
{
    assert(m_handle.is_valid());
    auto res = mesh().get_attribute_dimension(m_handle);
    assert(res > 0);
    return res;
}


} // namespace wmtk::function
