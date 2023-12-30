

#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
class MinEdgeLengthInvariant : public Invariant
{
public:
    // NOTE: this takes in the threshold squared rather than the threshold itself
    MinEdgeLengthInvariant(
        const Mesh& m,
        const MeshAttributeHandle<double>& coordinate,
        double threshold_squared);
    using Invariant::Invariant;
    bool before(const Simplex& t) const override;

private:
    const MeshAttributeHandle<double> m_coordinate_handle;
    double m_threshold_squared;
};
} // namespace wmtk
