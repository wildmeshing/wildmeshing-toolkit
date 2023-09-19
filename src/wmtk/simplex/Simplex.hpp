#pragma once

#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::simplex {

class Simplex
{
    PrimitiveType _ptype;
    Tuple _tuple;

public:
    Simplex(const PrimitiveType& ptype, const Tuple& t)
        : _ptype{ptype}
        , _tuple{t}
    {}

    PrimitiveType primitive_type() const { return _ptype; }
    long dimension() const { return get_simplex_dimension(_ptype); }
    const Tuple& tuple() const { return _tuple; }

    static Simplex vertex(const Tuple& t) { return Simplex(PrimitiveType::Vertex, t); }
    static Simplex edge(const Tuple& t) { return Simplex(PrimitiveType::Edge, t); }
    static Simplex face(const Tuple& t) { return Simplex(PrimitiveType::Face, t); }
    static Simplex tetrahedron(const Tuple& t) { return Simplex(PrimitiveType::Tetrahedron, t); }
};
} // namespace wmtk::simplex
