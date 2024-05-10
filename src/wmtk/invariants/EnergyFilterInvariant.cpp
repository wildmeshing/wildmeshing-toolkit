#include "EnergyFilterInvariant.hpp"
#include <wmtk/Mesh.hpp>

#include <wmtk/simplex/k_ring.hpp>

namespace wmtk {

EnergyFilterInvariant::EnergyFilterInvariant(
    const Mesh& m,
    const attribute::TypedAttributeHandle<char>& energy_filter_handle)
    : Invariant(m)
    , m_energy_filter_handle(energy_filter_handle)
{}

bool EnergyFilterInvariant::before(const simplex::Simplex& t) const
{
    assert(
        t.primitive_type() == PrimitiveType::Edge || t.primitive_type() == PrimitiveType::Vertex);
    auto accessor = mesh().create_const_accessor(m_energy_filter_handle);

    if (t.primitive_type() == PrimitiveType::Edge) {
        if (accessor.const_scalar_attribute(t.tuple()) == char(1) ||
            accessor.const_scalar_attribute(
                mesh().switch_tuple(t.tuple(), PrimitiveType::Vertex)) == char(1)) {
            return true;
        }
    } else {
        if (accessor.const_scalar_attribute(t.tuple()) == char(1)) {
            return true;
        }
    }

    return false;
}

} // namespace wmtk