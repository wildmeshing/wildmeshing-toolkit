#pragma once

#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
template <typename Derived>
class MeshCRTP;
namespace attribute {
template <typename T, typename MeshType, int Dim>
class Accessor;
}
} // namespace wmtk
namespace wmtk::simplex {

class Simplex
{
    friend class wmtk::Mesh;
    template <typename Derived>
    friend class wmtk::MeshCRTP;
    template <typename T, typename MeshType, int Dim>
    friend class attribute::Accessor;
    PrimitiveType m_primitive_type;
    Tuple m_tuple;
    // the mesh class can use this index value to cache/accelerate operations
    mutable int64_t m_index = -1;

    // private constructor mesh might want to use if it knows the ids beforehand
    Simplex(const PrimitiveType& ptype, const Tuple& t, int64_t index)
        : m_primitive_type{ptype}
        , m_tuple{t}
        , m_index(index)
    {}

public:
    Simplex(const PrimitiveType& ptype, const Tuple& t)
        : Simplex(ptype, t, -1)
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
