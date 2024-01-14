#include "EdgeValenceInvariant.hpp"

#include <wmtk/simplex/top_dimension_cofaces.hpp>

namespace wmtk::invariants {
EdgeValenceInvariant::EdgeValenceInvariant(const Mesh& m, int64_t valence)
    : Invariant(m,true,false,false)
    , m_valence(valence)
{}

bool EdgeValenceInvariant::before(const simplex::Simplex& simplex) const
{
    const Tuple& t = simplex.tuple();

    assert(simplex.primitive_type() == PrimitiveType::Edge);

    return simplex::top_dimension_cofaces_tuples(mesh(), simplex).size() == m_valence;
}
} // namespace wmtk::invariants
