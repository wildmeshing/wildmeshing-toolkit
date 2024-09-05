#include "Function.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::function {

Function::Function(Mesh& mesh, const attribute::MeshAttributeHandle& handle)
    : m_mesh(mesh)
    , m_handle(handle)
{
    assert(handle.holds<double>() || handle.holds<Rational>());
    assert(handle.is_same_mesh(m_mesh));
}

int64_t Function::embedded_dimension() const
{
    assert(m_handle.is_valid());

    auto res = m_handle.dimension();
    assert(res > 0);
    return res;
}


} // namespace wmtk::function
