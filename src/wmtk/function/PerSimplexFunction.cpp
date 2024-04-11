#include "PerSimplexFunction.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::function {
PerSimplexFunction::PerSimplexFunction(
    const Mesh& mesh,
    const PrimitiveType primitive_type,
    const attribute::MeshAttributeHandle& variable_attribute_handle)
    : m_handle(variable_attribute_handle)
    , m_mesh(mesh)
    , m_primitive_type(primitive_type)
{
    assert(variable_attribute_handle.is_same_mesh(m_mesh));
    assert(m_handle.holds<double>() || m_handle.holds<Rational>());
}


int64_t PerSimplexFunction::embedded_dimension() const
{
    if (m_handle.holds<double>()) {
        auto res = mesh().get_attribute_dimension(attribute_handle().as<double>());
        assert(res > 0);
        return res;
    }
    auto res = mesh().get_attribute_dimension(attribute_handle().as<Rational>());
    assert(res > 0);
    return res;
}
} // namespace wmtk::function
