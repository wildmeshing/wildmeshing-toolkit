#pragma once

#include <wmtk/multimesh/BoundaryChecker.hpp>
#include "Invariant.hpp"

namespace wmtk {
namespace invariants {
class InteriorSimplexInvariant : public Invariant
{
public:
    using Invariant::Invariant;
    InteriorSimplexInvariant(const Mesh& m, PrimitiveType pt);
    bool before(const simplex::Simplex& t) const override;

    void add_boundary(const Mesh& boundary_mesh);

private:
    PrimitiveType m_primitive_type;
    multimesh::BoundaryChecker m_boundary_checker;
};
} // namespace invariants
} // namespace wmtk
