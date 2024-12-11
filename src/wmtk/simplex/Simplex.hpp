#pragma once

#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;

namespace multimesh {
class MultiMeshManager;
}
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
    friend class multimesh::MultiMeshManager;
    template <typename Derived>
    friend class wmtk::MeshCRTP;
    template <typename T, typename MeshType, int Dim>
    friend class attribute::Accessor;

    friend class NavigatableSimplex;
    PrimitiveType m_primitive_type;
    Tuple m_tuple;

public:
    // the mesh class can use this index value to cache/accelerate operations
    Simplex(const PrimitiveType& ptype, const Tuple& t)
        : m_primitive_type{ptype}
        , m_tuple{t}
    {}


    Simplex() = default;
    // TODO: deprecate
    Simplex(const Mesh& m, const PrimitiveType& ptype, const Tuple& t);

    Simplex(const Simplex&) = default;
    Simplex(Simplex&&) = default;
    Simplex& operator=(const Simplex&) = default;
    Simplex& operator=(Simplex&&) = default;

    PrimitiveType primitive_type() const { return m_primitive_type; }
    int64_t dimension() const { return get_primitive_type_id(m_primitive_type); }
    const Tuple& tuple() const { return m_tuple; }

    // TODO: deprecate
    static Simplex vertex(const Mesh& m, const Tuple& t)
    {
        return Simplex(PrimitiveType::Vertex, t);
    }
    // TODO: deprecate
    static Simplex edge(const Mesh& m, const Tuple& t) { return Simplex(PrimitiveType::Edge, t); }
    // TODO: deprecate
    static Simplex face(const Mesh& m, const Tuple& t)
    {
        return Simplex(PrimitiveType::Triangle, t);
    }
    // TODO: deprecate
    static Simplex tetrahedron(const Mesh& m, const Tuple& t)
    {
        return Simplex(PrimitiveType::Tetrahedron, t);
    }

    static Simplex vertex(const Tuple& t) { return Simplex(PrimitiveType::Vertex, t); }
    static Simplex edge(const Tuple& t) { return Simplex(PrimitiveType::Edge, t); }
    static Simplex face(const Tuple& t) { return Simplex(PrimitiveType::Triangle, t); }
    static Simplex tetrahedron(const Tuple& t) { return Simplex(PrimitiveType::Tetrahedron, t); }

    // these operations are only internally defined if caching is enabled to make sure there's a
    // consistent semantic when simplex id caching is enabled vs not
};
} // namespace wmtk::simplex
