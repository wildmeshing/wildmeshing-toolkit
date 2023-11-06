#pragma once

#include "InteriorSimplexInvariant.hpp"

namespace wmtk {
namespace invariants {
class InteriorVertexInvariant : public InteriorSimplexInvariant
{
public:
    InteriorVertexInvariant(const Mesh& m);
};
} // namespace invariants
using InteriorVertexInvariant = invariants::InteriorVertexInvariant;
} // namespace wmtk
