#include "Cell.hpp"
#include <tuple>

namespace wmtk {
bool Cell::operator==(const Cell& o) const
{
    return m_dimension == o.m_dimension && m_tuple == o.m_tuple;
}

bool Cell::operator<(const Cell& o) const
{
    return std::tie(m_dimension, m_tuple) < std::tie(o.m_dimension, o.m_tuple);
}
} // namespace wmtk
