#include "PerSimplexFunction.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::function {
PerSimplexFunction::PerSimplexFunction(
    const Mesh& mesh,
    const PrimitiveType primitive_type,
    const attribute::MeshAttributeHandle<double>& variable_attribute_handle)
    : m_handle(variable_attribute_handle)
    , m_mesh(mesh)
    , m_primitive_type(primitive_type)
{}


int64_t PerSimplexFunction::embedded_dimension() const
{
    auto res = mesh().get_attribute_dimension(attribute_handle());
    assert(res > 0);
    return res;
}
} // namespace wmtk::function
