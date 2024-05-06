#pragma once

#if defined(WMTK_ENABLE_MULTIMESH)
#include <wmtk/multimesh/BoundaryChecker.hpp>
#endif
#include "Invariant.hpp"

namespace wmtk {
namespace invariants {
class InteriorSimplexInvariant : public Invariant
{
public:
    using Invariant::Invariant;
    InteriorSimplexInvariant(const Mesh& m, PrimitiveType pt);
    bool before(const simplex::Simplex& t) const override;

#if defined(WMTK_ENABLE_MULTIMESH)
    void add_boundary(const Mesh& boundary_mesh);
#endif

private:
    PrimitiveType m_primitive_type;
#if defined(WMTK_ENABLE_MULTIMESH)
    multimesh::BoundaryChecker m_boundary_checker;
#endif
};
} // namespace invariants
} // namespace wmtk
