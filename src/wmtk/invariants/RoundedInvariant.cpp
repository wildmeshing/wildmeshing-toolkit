#include "RoundedInvariant.hpp"
#include <wmtk/Mesh.hpp>

#include <wmtk/simplex/k_ring.hpp>

namespace wmtk {

RoundedInvariant::RoundedInvariant(
    const Mesh& m,
    const attribute::TypedAttributeHandle<Rational>& coordinate,
    bool inverse_flag)
    : Invariant(m)
    , m_coordinate_handle(coordinate)
    , inverse(inverse_flag)
{}

bool RoundedInvariant::before(const simplex::Simplex& t) const
{
    auto accessor = mesh().create_const_accessor(m_coordinate_handle);
    int dim = mesh().get_attribute_dimension(m_coordinate_handle);

    bool rounded = true;
    for (int i = 0; i < dim; ++i) {
        if (!accessor.const_vector_attribute(t.tuple())[i].is_rounded()) rounded = false;
    }

    if (rounded && !inverse) {
        return true;
    } else if (!rounded && inverse) {
        return true;
    }

    return true;
}

} // namespace wmtk