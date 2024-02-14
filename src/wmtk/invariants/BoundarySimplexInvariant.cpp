#include "BoundarySimplexInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {
BoundarySimplexInvariant::BoundarySimplexInvariant(const Mesh& m, PrimitiveType pt)
    : Invariant(m, true, false, false)
    , m_primitive_type(pt)
{}
bool BoundarySimplexInvariant::before(const simplex::Simplex& t) const
{
    const bool result = mesh().is_boundary(m_primitive_type, t.tuple());
    return result;
}
} // namespace wmtk::invariants
