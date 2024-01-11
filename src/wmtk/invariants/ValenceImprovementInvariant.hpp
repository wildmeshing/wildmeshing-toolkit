#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk::invariants {
class ValenceImprovementInvariant : public Invariant
{
public:
    ValenceImprovementInvariant(const Mesh& m);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& t) const override;

    static std::pair<int64_t, int64_t> valence_change(const Mesh& mesh, const simplex::Simplex& s);
};
} // namespace wmtk::invariants
