#pragma once

#include "Invariant.hpp"

namespace wmtk::invariants {
class ValenceImprovementInvariant : public Invariant
{
public:
    ValenceImprovementInvariant(const Mesh& m);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& t) const override;
};
} // namespace wmtk::invariants
