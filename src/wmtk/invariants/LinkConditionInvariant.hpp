#pragma once

#include "Invariant.hpp"

namespace wmtk {
namespace invariants {
class LinkConditionInvariant : public Invariant
{
public:
    LinkConditionInvariant(const Mesh& m);
    bool before(const simplex::Simplex& t) const override;
};
} // namespace invariants
using LinkConditionInvariant = invariants::LinkConditionInvariant;
} // namespace wmtk
