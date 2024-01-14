#include "InteriorSimplexInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {
InteriorSimplexInvariant::InteriorSimplexInvariant(const Mesh& m, PrimitiveType pt)
    : Invariant(m)
    , m_primitive_type(pt)
{}
bool InteriorSimplexInvariant::before(const simplex::Simplex& t) const
{
    const bool result = !mesh().is_boundary(m_primitive_type, t.tuple());
    return result;
}
} // namespace wmtk::invariants
