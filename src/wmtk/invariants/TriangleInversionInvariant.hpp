#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk {
class TriangleInversionInvariant : public MeshInvariant
{
public:
    // NOTE: this takes in the threshold squared rather than the threshold itself
    TriangleInversionInvariant(const Mesh& m, const MeshAttributeHandle<double>& uv_coordinate);
    using MeshInvariant::MeshInvariant;
    bool after(PrimitiveType type, const std::vector<Tuple>& t) const override;

private:
    const MeshAttributeHandle<double> m_uv_coordinate_handle;
};
} // namespace wmtk
