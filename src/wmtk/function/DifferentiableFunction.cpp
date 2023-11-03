#include "DifferentiableFunction.hpp"
#include <wmtk/Mesh.hpp>
namespace wmtk::function {

//DifferentiableFunction::DifferentiableFunction(
//    const MeshAttributeHandle<double>& attribute_handle)
//    : m_attribute_handle(attribute_handle)
//{}
//
//
//const MeshAttributeHandle<double>& DifferentiableFunction::get_coordinate_attribute_handle() const
//{
//    return m_coordinate_attribute_handle;
//}
long DifferentiableFunction::embedded_dimension() const
{
    return mesh().get_attribute_dimension(get_coordinate_attribute_handle());
}
} // namespace wmtk::function
