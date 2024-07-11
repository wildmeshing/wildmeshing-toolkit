#pragma once

#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
class EnergyFilterInvariant : public Invariant
{
public:
    EnergyFilterInvariant(
        const Mesh& m,
        const attribute::TypedAttributeHandle<char>& energy_filter_handle);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& t) const override;

private:
    const attribute::TypedAttributeHandle<char> m_energy_filter_handle;
};
} // namespace wmtk