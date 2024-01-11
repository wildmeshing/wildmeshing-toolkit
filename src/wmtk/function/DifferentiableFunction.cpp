
#include "Function.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::function {

Mesh& Function::mesh()
{
    return const_cast<Mesh&>(const_cast<const Function*>(this)->mesh());
}
Function::Function(Mesh& mesh, const attribute::MeshAttributeHandle& handle)
    : Function(handle)
{
    assert(handle.is_same_mesh(m_mesh));
}

Function::Function(const attribute::MeshAttributeHandle& handle)
    : m_handle(handle)
{
    assert(handle.holds<double>());
}

int64_t Function::variable_embedded_dimension() const
{
    assert(m_handle.is_valid());
    int64_t res = mesh().get_attribute_dimension(m_handle.as<double>());
    assert(res > 0);
    return res;
}

PrimitiveType Function::variable_attribute_type() const
{
    return m_variable_attribute_handle.primitive_type();
};
const attribute::MeshAttributeHandle& Function::variable_attribute_handle() const
{
    return m_variable_attribute_handle;
}

} // namespace wmtk::function
