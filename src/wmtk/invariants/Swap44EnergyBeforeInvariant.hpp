#pragma once

#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
class Swap44EnergyBeforeInvariant : public Invariant
{
public:
    Swap44EnergyBeforeInvariant(
        const Mesh& m,
        const attribute::TypedAttributeHandle<Rational>& coordinate);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& t) const override;

private:
    const attribute::TypedAttributeHandle<Rational> m_coordinate_handle;
};
} // namespace wmtk
