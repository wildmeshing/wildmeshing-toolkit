#pragma once

#include "Invariant.hpp"

namespace wmtk {

namespace invariants {

class NoBoundaryCollapseToInteriorInvariant : public Invariant
{
public:
    using Invariant::Invariant;
    NoBoundaryCollapseToInteriorInvariant(const Mesh& m);
    bool before(const simplex::Simplex& t) const override;
};
} // namespace invariants

} // namespace wmtk