#pragma once

#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
class Swap56EnergyBeforeInvariantDouble : public Invariant
{
public:
    Swap56EnergyBeforeInvariantDouble(
        const Mesh& m,
        const attribute::TypedAttributeHandle<double>& coordinate,
        int64_t collapse_index,
        double eps = 1.0001);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& t) const override;

private:
    const attribute::TypedAttributeHandle<double> m_coordinate_handle;
    int64_t m_collapse_index;
    const double m_eps;
};

} // namespace wmtk
