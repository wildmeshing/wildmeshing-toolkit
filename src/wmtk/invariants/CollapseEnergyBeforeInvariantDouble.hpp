#pragma once

#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk::invariants {
class CollapseEnergyBeforeInvariantDouble : public Invariant
{
public:
    CollapseEnergyBeforeInvariantDouble(
        const Mesh& m,
        const attribute::TypedAttributeHandle<double>& coordinate,
        const attribute::TypedAttributeHandle<double>& energy,
        int64_t collapse_type = 0);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& s) const override;

private:
    const attribute::TypedAttributeHandle<double> m_coordinate_handle;
    const attribute::TypedAttributeHandle<double> m_energy_handle;
    const int64_t m_collapse_type; // 0: collapse to v0, 1: collapse to v1, 2: collapse to midpoint
};
} // namespace wmtk::invariants
