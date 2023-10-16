#include "DifferentiableFunction.hpp"
namespace wmtk::function {

DifferentiableFunction::DifferentiableFunction(
    const Mesh& mesh,
    const MeshAttributeHandle<double>& vertex_attribute_handle)
    : Function(mesh)
    , m_vertex_attribute_handle(vertex_attribute_handle)
{}

const MeshAttributeHandle<double> DifferentiableFunction::get_vertex_attribute_handle() const
{
    return m_vertex_attribute_handle;
}
long DifferentiableFunction::embedded_dimension() const
{
    return mesh().get_attribute_dimension(get_vertex_attribute_handle());
}
} // namespace wmtk::function
