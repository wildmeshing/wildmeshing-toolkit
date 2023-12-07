#include "Cell.hpp"
#include <tuple>

namespace wmtk {

Cell::Cell(const long& dimension, const Tuple& t)
    : m_dimension{dimension}
    , m_tuple{t}
{}

Cell::Cell(const Simplex& simplex)
    : m_dimension{simplex.dimension()}
    , m_tuple{simplex.tuple()}
{}


long Cell::dimension() const
{
    return m_dimension;
}
const Tuple& Cell::tuple() const
{
    return m_tuple;
}

Cell Cell::vertex(const Tuple& t)
{
    return Cell(0, t);
}
Cell Cell::edge(const Tuple& t)
{
    return Cell(1, t);
}
Cell Cell::face(const Tuple& t)
{
    return Cell(2, t);
}
Cell Cell::tetrahedron(const Tuple& t)
{
    return Cell(3, t);
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
