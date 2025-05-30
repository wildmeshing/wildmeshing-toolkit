#pragma once

#include "Invariant.hpp"

namespace wmtk {
namespace invariants {
class InteriorSimplexInvariant : public Invariant
{
public:
    using Invariant::Invariant;
    InteriorSimplexInvariant(const Mesh& m, PrimitiveType pt);
    bool before(const simplex::Simplex& t) const override;

private:
    PrimitiveType m_primitive_type;
};
} // namespace invariants
} // namespace wmtk
