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
};
} // namespace wmtk::invariants
