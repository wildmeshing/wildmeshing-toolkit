#pragma once

#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk::invariants {
class CollapseSoftEnergyBeforeInvariant : public Invariant
{
public:
    CollapseSoftEnergyBeforeInvariant(
        const Mesh& m,
        const attribute::TypedAttributeHandle<Rational>& coordinate,
        const attribute::TypedAttributeHandle<double>& energy,
        const attribute::TypedAttributeHandle<double>& edge_length,
        const attribute::TypedAttributeHandle<double>& target_edge_length,
        int64_t collapse_type = 0,
        double eps = 0.1);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& s) const override;

private:
    const attribute::TypedAttributeHandle<Rational> m_coordinate_handle;
    const attribute::TypedAttributeHandle<double> m_energy_handle;
    const attribute::TypedAttributeHandle<double> m_edge_length_handle;
    const attribute::TypedAttributeHandle<double> m_target_edge_length_handle;
    const int64_t m_collapse_type; // 0: collapse to v0, 1: collapse to v1, 2: collapse to midpoint
    const double m_eps;
};
} // namespace wmtk::invariants
