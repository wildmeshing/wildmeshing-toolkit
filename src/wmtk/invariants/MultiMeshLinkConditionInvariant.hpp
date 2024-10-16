#pragma once

#include "Invariant.hpp"

namespace wmtk {
namespace invariants {
class MultiMeshLinkConditionInvariant : public Invariant
{
public:
    MultiMeshLinkConditionInvariant(const Mesh& m);
    bool before(const simplex::Simplex& t) const override;
};
} // namespace invariants
using MultiMeshLinkConditionInvariant = invariants::MultiMeshLinkConditionInvariant;
} // namespace wmtk
