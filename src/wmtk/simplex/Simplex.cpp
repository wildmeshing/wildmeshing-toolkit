#include "Simplex.hpp"

#include <tuple>
#include <wmtk/Mesh.hpp>
namespace wmtk::simplex {

Simplex::Simplex(const Mesh& m, const PrimitiveType& ptype, const Tuple& t)
#if defined(WMTK_ENABLE_SIMPLEX_ID_CACHING)
    : Simplex(ptype, t, m.id(t, ptype))
#else
    : Simplex(ptype, t)
#endif
{}

#if defined(WMTK_ENABLE_SIMPLEX_ID_CACHING)
bool Simplex::operator==(const Simplex& o) const
{
    return m_primitive_type == o.m_primitive_type && m_index == o.m_index;
}

bool Simplex::operator<(const Simplex& o) const
{
    return std::tie(m_primitive_type, m_index) < std::tie(o.m_primitive_type, o.m_index);
}
#endif
} // namespace wmtk::simplex
