#include "Simplex.hpp"

#include <tuple>
#include <wmtk/Mesh.hpp>
namespace wmtk::simplex {

Simplex::Simplex(const Mesh& m, const PrimitiveType& ptype, const Tuple& t)
    : Simplex(ptype, t, m.id(t, ptype))
{}

bool Simplex::operator==(const Simplex& o) const
{
    return m_primitive_type == o.m_primitive_type && m_index == o.m_index;
}

bool Simplex::operator<(const Simplex& o) const
{
    return std::tie(m_primitive_type, m_index) < std::tie(o.m_primitive_type, o.m_index);
}
} // namespace wmtk::simplex
