#pragma once

#include <wmtk/simplex/Simplex.hpp>
#include "Tuple.hpp"

namespace wmtk {

class Cell
{
    Tuple m_tuple;
    long m_dimension;

public:
    Cell(const Tuple& t, long dimension);
    Cell(const simplex::Simplex& simplex);
    Cell(const Tuple& t, PrimitiveType pt);


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
