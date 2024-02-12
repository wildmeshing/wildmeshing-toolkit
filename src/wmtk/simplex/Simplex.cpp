#include "Simplex.hpp"

#include <tuple>
#include <wmtk/Mesh.hpp>

namespace wmtk::simplex {

Simplex::Simplex(const Mesh& mesh, const PrimitiveType& ptype, const Tuple& t)
    : m_primitive_type{ptype}
    , m_tuple{t}
    , m_id(mesh.id(t, ptype))
{}

bool Simplex::operator==(const Simplex& o) const
{
    return m_primitive_type == o.m_primitive_type && m_tuple == o.m_tuple;
}

bool Simplex::operator<(const Simplex& o) const
{
    return std::tie(m_primitive_type, m_tuple) < std::tie(o.m_primitive_type, o.m_tuple);
}
} // namespace wmtk::simplex
