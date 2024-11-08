#pragma once

#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
class Swap2dEdgeLengthInvariant : public Invariant
{
public:
    Swap2dEdgeLengthInvariant(
        const Mesh& m,
        const attribute::TypedAttributeHandle<double>& coordinate);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& t) const override;

private:
    const attribute::TypedAttributeHandle<double> m_coordinate_handle;
};
} // namespace wmtk
