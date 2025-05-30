#include "InteriorSimplexInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {
InteriorSimplexInvariant::InteriorSimplexInvariant(const Mesh& m, PrimitiveType pt)
    : Invariant(m, true, false, false)
    , m_primitive_type(pt)
{}

bool InteriorSimplexInvariant::before(const simplex::Simplex& t) const
{
    return !mesh().is_boundary(t);
}

} // namespace wmtk::invariants
