#include "Swap2dEdgeLengthInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/function/utils/amips.hpp>
#include <wmtk/utils/orient.hpp>

namespace wmtk {
Swap2dEdgeLengthInvariant::Swap2dEdgeLengthInvariant(
    const Mesh& m,
    const attribute::TypedAttributeHandle<double>& coordinate)
    : Invariant(m, true, false, false)
    , m_coordinate_handle(coordinate)
{}

bool Swap2dEdgeLengthInvariant::before(const simplex::Simplex& t) const
{
    assert(m.top_cell_dimension() == 2);
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;

    auto accessor = mesh().create_const_accessor(m_coordinate_handle);

    // get the coords of the vertices
    // input face end points
    const Tuple v0 = t.tuple();
    const Tuple v1 = mesh().switch_tuple(v0, PV);
    // other 2 vertices
    const Tuple v2 = mesh().switch_tuples(v0, {PE, PV});
    const Tuple v3 = mesh().switch_tuples(v0, {PF, PE, PV});

    const double length_old =
        (accessor.const_vector_attribute(v0) - accessor.const_vector_attribute(v1)).norm();
    const double length_new =
        (accessor.const_vector_attribute(v2) - accessor.const_vector_attribute(v3)).norm();

    return length_new > length_old;
}

} // namespace wmtk
