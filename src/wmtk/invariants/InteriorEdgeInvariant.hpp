#pragma once

#include "InteriorSimplexInvariant.hpp"

namespace wmtk {
namespace invariants {
class InteriorEdgeInvariant : public InteriorSimplexInvariant
{
public:
    InteriorEdgeInvariant(const Mesh& m);
};
} // namespace invariants
using InteriorEdgeInvariant = invariants::InteriorEdgeInvariant;
} // namespace wmtk
