#include "InteriorSimplexInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk::invariants {
InteriorSimplexInvariant::InteriorSimplexInvariant(const Mesh& m, PrimitiveType pt)
    : MeshInvariant(m)
    , m_primitive_type(pt)
{}
bool InteriorSimplexInvariant::before(const Tuple& t) const
{
    const bool result = !mesh().is_boundary(t, m_primitive_type);
    return result;
}
} // namespace wmtk::invariants
