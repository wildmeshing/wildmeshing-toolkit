#pragma once

#include "Simplex.hpp"
#include "Tuple.hpp"

namespace wmtk {

class Cell
{
    long m_dimension;
    Tuple m_tuple;

public:
    Cell(const long& dimension, const Tuple& t);
    Cell(const Simplex& simplex);


    long dimension() const;
    const Tuple& tuple() const;

    static Cell vertex(const Tuple& t);
    static Cell edge(const Tuple& t);
    static Cell face(const Tuple& t);
    static Cell tetrahedron(const Tuple& t);

    bool operator==(const Cell& o) const;
    bool operator<(const Cell& o) const;
};
} // namespace wmtk
