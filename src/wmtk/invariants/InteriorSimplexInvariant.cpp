#include "InteriorSimplexInvariant.hpp"
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/Mesh.hpp>

namespace wmtk::invariants {
InteriorSimplexInvariant::InteriorSimplexInvariant(const Mesh& m, PrimitiveType pt)
    : Invariant(m)
    , m_primitive_type(pt)
{}
bool InteriorSimplexInvariant::before(const simplex::Simplex& t) const
{
    const bool result = !mesh().is_boundary(t.tuple(), m_primitive_type);
    return result;
}
} // namespace wmtk::invariants
