#include "RoundedInvariant.hpp"
#include <wmtk/Mesh.hpp>

#include <wmtk/simplex/k_ring.hpp>

namespace wmtk {

RoundedInvariant::RoundedInvariant(
    const Mesh& m,
    const attribute::TypedAttributeHandle<Rational>& coordinate)
    : Invariant(m)
    , m_coordinate_handle(coordinate)
{}

bool RoundedInvariant::before(const simplex::Simplex& t) const
{
    auto accessor = mesh().create_const_accessor(m_coordinate_handle);
    int dim = mesh().get_attribute_dimension(m_coordinate_handle);

    for (int i = 0; i < dim; ++i) {
        if (!accessor.const_vector_attribute(t.tuple())[i].is_rounded()) return false;
    }

    return true;
}

} // namespace wmtk