#include "Simplex.hpp"
#include <tuple>
namespace wmtk::simplex {
bool Simplex::operator==(const Simplex& o) const
{
    return m_primitive_type == o.m_primitive_type && m_tuple == o.m_tuple;
}

bool Simplex::operator<(const Simplex& o) const
{
    return std::tie(m_primitive_type, m_tuple) < std::tie(o.m_primitive_type, o.m_tuple);
}
} // namespace wmtk::simplex
