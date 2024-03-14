#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
class Swap44EnergyBeforeInvariant : public Invariant
{
public:
    Swap44EnergyBeforeInvariant(const Mesh& m, const TypedAttributeHandle<double>& coordinate);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& t) const override;

private:
    const TypedAttributeHandle<double> m_coordinate_handle;
};
} // namespace wmtk