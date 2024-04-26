#include "InteriorSimplexInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::invariants {
InteriorSimplexInvariant::InteriorSimplexInvariant(const Mesh& m, PrimitiveType pt)
    : Invariant(m, true, false, false)
    , m_primitive_type(pt)
#if defined(WMTK_ENABLE_MULTIMESH)
    , m_boundary_checker(m)
#endif
{}

bool InteriorSimplexInvariant::before(const simplex::Simplex& t) const
{
#if defined(WMTK_ENABLE_MULTIMESH)
    const bool result =
        !m_boundary_checker.is_boundary(mesh(), simplex::Simplex(m_primitive_type, t.tuple()));
#else
    const bool result = mesh().is_boundary(t);
#endif
    return result;
}


#if defined(WMTK_ENABLE_MULTIMESH)
void InteriorSimplexInvariant::add_boundary(const Mesh& boundary_mesh)
{
    m_boundary_checker.add_mesh(boundary_mesh);
}
#endif
} // namespace wmtk::invariants
