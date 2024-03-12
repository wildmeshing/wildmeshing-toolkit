#include "InteriorSimplexInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {
InteriorSimplexInvariant::InteriorSimplexInvariant(const Mesh& m, PrimitiveType pt)
    : Invariant(m, true, false, false)
    , m_primitive_type(pt)
    , m_boundary_checker(m)
{}

bool InteriorSimplexInvariant::before(const simplex::Simplex& t) const
{
    const bool result =
        !m_boundary_checker.is_boundary(mesh(), simplex::Simplex(m_primitive_type, t.tuple()));
    return result;
}


void InteriorSimplexInvariant::add_boundary(const Mesh& boundary_mesh)
{
    m_boundary_checker.add_mesh(boundary_mesh);
}
} // namespace wmtk::invariants
