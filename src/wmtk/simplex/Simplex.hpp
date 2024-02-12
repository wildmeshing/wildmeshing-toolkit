#pragma once

#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
}

namespace wmtk::simplex {

class Simplex
{
    PrimitiveType m_primitive_type;
    Tuple m_tuple;
    int64_t m_id;

public:
    Simplex(const Mesh& m, const PrimitiveType& ptype, const Tuple& t);
    Simplex(const Simplex&) = default;
    Simplex(Simplex&&) = default;
    Simplex& operator=(const Simplex&) = default;
    Simplex& operator=(Simplex&&) = default;

    PrimitiveType primitive_type() const { return m_primitive_type; }
    int64_t dimension() const { return get_primitive_type_id(m_primitive_type); }
    const Tuple& tuple() const { return m_tuple; }

    static Simplex vertex(const Mesh& m, const Tuple& t)
    {
        return Simplex(m, PrimitiveType::Vertex, t);
    }
    static Simplex edge(const Mesh& m, const Tuple& t)
    {
        return Simplex(m, PrimitiveType::Edge, t);
    }
    static Simplex face(const Mesh& m, const Tuple& t)
    {
        return Simplex(m, PrimitiveType::Triangle, t);
    }
    static Simplex tetrahedron(const Mesh& m, const Tuple& t)
    {
        return Simplex(m, PrimitiveType::Tetrahedron, t);
    }

    bool operator==(const Simplex& o) const;
    bool operator<(const Simplex& o) const;

    int64_t id() const { return m_id; }
};
} // namespace wmtk::simplex
