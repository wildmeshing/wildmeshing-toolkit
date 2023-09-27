

#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk {
class MinEdgeLengthInvariant: public MeshInvariant
{
    public:
    // NOTE: this takes in the threshold squared rather than the threshold itself
    MinEdgeLengthInvariant(const Mesh& m, const MeshAttributeHandle<double>& coordinate, double threshold_squared); 
    using MeshInvariant::MeshInvariant;
    bool before(const Tuple& t) const override;

    private:
    const MeshAttributeHandle<double> m_coordinate_handle;
    double m_threshold_squared;
};
} // namespace wmtk
