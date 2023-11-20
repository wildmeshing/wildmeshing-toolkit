#pragma once

#include "Simplex.hpp"
#include "Tuple.hpp"

namespace wmtk {

class Cell
{
    long m_dimension;
    Tuple m_tuple;

public:
    Cell(const long& dimension, const Tuple& t)
        : m_dimension{dimension}
        , m_tuple{t}
    {}
    Cell(const Simplex& simplex)
        : m_dimension{simplex.dimension()}
        , m_tuple{simplex.tuple()}
    {}


    long dimension() const { return m_dimension; }
    const Tuple& tuple() const { return m_tuple; }

    static Cell vertex(const Tuple& t) { return Cell(0, t); }
    static Cell edge(const Tuple& t) { return Cell(1, t); }
    static Cell face(const Tuple& t) { return Cell(2, t); }
    static Cell tetrahedron(const Tuple& t) { return Cell(3, t); }

    bool operator==(const Cell& o) const;
    bool operator<(const Cell& o) const;
};
} // namespace wmtk
