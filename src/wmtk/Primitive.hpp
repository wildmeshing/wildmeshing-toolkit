#pragma once

#include "PrimitiveType.hpp"
#include "Tuple.hpp"

namespace wmtk {
    class Cell;
    namespace simplex {
        class Simplex;
    }

class Primitive
{
    PrimitiveType m_primitive_type;
    Tuple m_tuple;

public:
    Primitive(const PrimitiveType& primitive_type, const Tuple& t);
    Primitive(const simplex::Simplex& simplex);
    Primitive(const Cell& cell);

    PrimitiveType primitive_type() const;
    const Tuple& tuple() const;

    static Primitive vertex(const Tuple& t);
    static Primitive edge(const Tuple& t);
    static Primitive face(const Tuple& t);
    static Primitive tetrahedron(const Tuple& t);
    static Primitive halfedge(const Tuple& t);

    bool operator==(const Primitive& o) const;
    bool operator<(const Primitive& o) const;
};
} // namespace wmtk
