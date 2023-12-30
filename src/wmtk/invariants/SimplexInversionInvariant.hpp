#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
class SimplexInversionInvariant : public Invariant
{
public:
    SimplexInversionInvariant(const Mesh& m, const MeshAttributeHandle<double>& coordinate);
    using Invariant::Invariant;

    /**
     *  we assume with local vid order (v0,v1,v2,v3) has positive volume (orient3d(v0,v1,v2,v3)>0)
     a CLOCKWISE input tuple will have the same orientation as the local vid order
     */
    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

private:
    const MeshAttributeHandle<double> m_coordinate_handle;
};
} // namespace wmtk