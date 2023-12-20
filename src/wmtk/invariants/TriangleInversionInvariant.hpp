#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk {
class TriangleInversionInvariant : public Invariant
{
public:
    // NOTE: this takes in the threshold squared rather than the threshold itself
    TriangleInversionInvariant(const Mesh& m, const MeshAttributeHandle<double>& uv_coordinate);
    using Invariant::Invariant;
    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

private:
    const MeshAttributeHandle<double> m_uv_coordinate_handle;
};
} // namespace wmtk
