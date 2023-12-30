#include "Cell.hpp"
#include <tuple>

namespace wmtk {

Cell::Cell(const Tuple& t, int64_t dimension)
    : m_tuple{t}
    , m_dimension{dimension}
{}

Cell::Cell(const simplex::Simplex& simplex)
    : m_tuple{simplex.tuple()}
    , m_dimension{simplex.dimension()}
{}

Cell::Cell(const Tuple& t, PrimitiveType pt)
    : m_tuple{t}
    , m_dimension(get_primitive_type_id(pt))
{}

int64_t Cell::dimension() const
{
    return m_dimension;
}
const Tuple& Cell::tuple() const
{
    return m_tuple;
}

Cell Cell::vertex(const Tuple& t)
{
    return Cell(t, 0);
}
Cell Cell::edge(const Tuple& t)
{
    return Cell(t, 1);
}
Cell Cell::face(const Tuple& t)
{
    return Cell(t, 2);
}
Cell Cell::tetrahedron(const Tuple& t)
{
    return Cell(t, 3);
}

bool Cell::operator==(const Cell& o) const
{
    return m_dimension == o.m_dimension && m_tuple == o.m_tuple;
}

bool Cell::operator<(const Cell& o) const
{
    return std::tie(m_dimension, m_tuple) < std::tie(o.m_dimension, o.m_tuple);
}
} // namespace wmtk
