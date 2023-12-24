#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk::invariants {
class ValenceImprovementInvariant : public Invariant
{
public:
    ValenceImprovementInvariant(const Mesh& m);
    using Invariant::Invariant;

    bool before(const Simplex& t) const override;

private:
    const MeshAttributeHandle<double> m_uv_coordinate_handle;
};
} // namespace wmtk::invariants
