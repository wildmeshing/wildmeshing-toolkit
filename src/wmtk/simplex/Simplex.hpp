#pragma once

#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::simplex {

class Simplex
{
    PrimitiveType m_primitive_type;
    Tuple m_tuple;

public:
    Simplex(const PrimitiveType& ptype, const Tuple& t)
        : m_primitive_type{ptype}
        , m_tuple{t}
    {}
    Simplex(const Simplex&) = default;
    Simplex(Simplex&&) = default;
    Simplex& operator=(const Simplex&) = default;
    Simplex& operator=(Simplex&&) = default;

    PrimitiveType primitive_type() const { return m_primitive_type; }
    int64_t dimension() const { return get_primitive_type_id(m_primitive_type); }
    const Tuple& tuple() const { return m_tuple; }

    static Simplex vertex(const Tuple& t) { return Simplex(PrimitiveType::Vertex, t); }
    static Simplex edge(const Tuple& t) { return Simplex(PrimitiveType::Edge, t); }
    static Simplex face(const Tuple& t) { return Simplex(PrimitiveType::Triangle, t); }
    static Simplex tetrahedron(const Tuple& t) { return Simplex(PrimitiveType::Tetrahedron, t); }

    bool operator==(const Simplex& o) const;
    bool operator<(const Simplex& o) const;
};
} // namespace wmtk::simplex
