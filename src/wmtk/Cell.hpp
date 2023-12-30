#pragma once

#include <wmtk/simplex/Simplex.hpp>
#include "Tuple.hpp"

namespace wmtk {

class Cell
{
    Tuple m_tuple;
    int64_t m_dimension;

public:
    Cell(const Tuple& t, int64_t dimension);
    Cell(const simplex::Simplex& simplex);
    Cell(const Tuple& t, PrimitiveType pt);


    int64_t dimension() const;
    const Tuple& tuple() const;

    static Cell vertex(const Tuple& t);
    static Cell edge(const Tuple& t);
    static Cell face(const Tuple& t);
    static Cell tetrahedron(const Tuple& t);

    bool operator==(const Cell& o) const;
    bool operator<(const Cell& o) const;
};
} // namespace wmtk
