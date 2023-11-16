#include "DifferentiableFunction.hpp"
#include <wmtk/Mesh.hpp>
namespace wmtk::function {

long DifferentiableFunction::embedded_dimension() const
{
    return mesh().get_attribute_dimension(get_coordinate_attribute_handle());
}
} // namespace wmtk::function
