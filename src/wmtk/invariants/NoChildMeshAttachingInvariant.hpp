#pragma once

#include "Invariant.hpp"

namespace wmtk {
namespace invariants {
class NoChildMeshAttachingInvariant : public Invariant
{
public:
    using Invariant::Invariant;
    NoChildMeshAttachingInvariant(const Mesh& m);
    bool before(const simplex::Simplex& t) const override;
};
} // namespace invariants
} // namespace wmtk
