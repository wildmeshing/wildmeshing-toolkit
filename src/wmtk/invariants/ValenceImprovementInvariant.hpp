#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
    class TriMesh;
}
namespace wmtk::invariants {
class ValenceImprovementInvariant : public Invariant
{
public:
    ValenceImprovementInvariant(const TriMesh& m);

    bool before(const simplex::Simplex& t) const override;
};
} // namespace wmtk::invariants
