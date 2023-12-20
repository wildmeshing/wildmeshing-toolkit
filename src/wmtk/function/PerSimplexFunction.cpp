#include "PerSimplexFunction.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::function {
PerSimplexFunction::PerSimplexFunction(
    const Mesh& mesh,
    const attribute::MeshAttributeHandle<double>& variable_attribute_handle)
    : m_coordinate_attribute_handle(variable_attribute_handle)
    , m_mesh(mesh)
{}


long PerSimplexFunction::embedded_dimension() const
{
    return mesh().get_attribute_dimension(get_coordinate_attribute_handle());
}
} // namespace wmtk::function
