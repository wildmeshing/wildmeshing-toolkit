#pragma once

#include "Invariant.hpp"

namespace wmtk {
namespace invariants {
class BoundarySimplexInvariant : public Invariant
{
public:
    using Invariant::Invariant;
    BoundarySimplexInvariant(const Mesh& m, PrimitiveType pt);
    bool before(const simplex::Simplex& t) const override;

private:
    PrimitiveType m_primitive_type;
};
} // namespace invariants
} // namespace wmtk
