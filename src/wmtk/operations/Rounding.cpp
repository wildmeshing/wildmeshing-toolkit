#include "Rounding.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::operations {

Rounding::Rounding(Mesh& m, TypedAttributeHandle<Rational>& coordinate)
    : AttributesUpdate(m)
    , m_coordinate_handle(coordinate)
{}

std::vector<simplex::Simplex> Rounding::execute(const simplex::Simplex& simplex)
{
    auto accessor = mesh().create_accessor(m_coordinate_handle);

    auto pos = accessor.vector_attribute(simplex.tuple());
    for (int i = 0; i < mesh().get_attribute_dimension(m_coordinate_handle); ++i) {
        pos[i].round();
    }
    accessor.vector_attribute(simplex.tuple()) = pos;

    return AttributesUpdate::execute(simplex);
}

} // namespace wmtk::operations