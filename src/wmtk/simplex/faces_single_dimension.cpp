#include "faces_single_dimension.hpp"
#include <cassert>
constexpr wmtk::PrimitiveType PV = wmtk::PrimitiveType::Vertex;
constexpr wmtk::PrimitiveType PE = wmtk::PrimitiveType::Edge;
constexpr wmtk::PrimitiveType PF = wmtk::PrimitiveType::Triangle;
constexpr wmtk::PrimitiveType PT = wmtk::PrimitiveType::Tetrahedron;

namespace wmtk::simplex {
std::vector<Tuple> vertices(const Mesh& m, const Simplex& simplex)
{
    if (simplex.primitive_type() == PrimitiveType::Vertex) {
        return {};
    }

    const Tuple v0 = simplex.tuple();
    const Tuple v1 = m.switch_tuple(v0, PV);

    if (simplex.primitive_type() == PrimitiveType::Edge) {
        return {v0, v1};
    }

    const Tuple v2 = m.switch_tuples(v0,{PE,PV});

    if (simplex.primitive_type() == PrimitiveType::Triangle) {
        return {v0, v1, v2};
    }

    const Tuple v3 = m.switch_tuples(v0,{PF,PE,PV});

    if (simplex.primitive_type() == PrimitiveType::Tetrahedron) {
        return {v0, v1, v2, v3};
    }

    throw std::runtime_error("unknown primitive type");
}

std::vector<Tuple> edges(const Mesh& m, const Simplex& simplex)
{
    if (simplex.primitive_type() == PrimitiveType::Vertex ||
        simplex.primitive_type() == PrimitiveType::Edge) {
        return {};
    }

    const Tuple e0 = simplex.tuple();
    const Tuple e1 = m.switch_tuples(e0,{PV,PE});
    const Tuple e2 = m.switch_tuples(e0,{PE,PV});

    if (simplex.primitive_type() == PrimitiveType::Triangle) {
        return {e0, e1, e2};
    }

    const Tuple e3 = m.switch_tuples(e0,{PF,PE});
    const Tuple e4 = m.switch_tuples(e1,{PF,PE});
    const Tuple e5 = m.switch_tuples(e2,{PF,PE});

    if (simplex.primitive_type() == PrimitiveType::Tetrahedron) {
        return {e0, e1, e2, e3, e4, e5};
    }

    throw std::runtime_error("unknown primitive type");
}

std::vector<Tuple> faces(const Mesh& m, const Simplex& simplex)
{
    if (simplex.primitive_type() == PrimitiveType::Vertex ||
        simplex.primitive_type() == PrimitiveType::Edge ||
        simplex.primitive_type() == PrimitiveType::Triangle) {
        return {};
    }

    const Tuple f0 = simplex.tuple();
    const Tuple f1 = m.switch_tuples(f0,{PF,PE});
    const Tuple f2 = m.switch_tuples(f0,{PV,PE,PF,PE});
    const Tuple f3 = m.switch_tuples(f0,{PE,PV,PF,PE});

    if (simplex.primitive_type() == PrimitiveType::Tetrahedron) {
        return {f0, f1, f2, f3};
    }

    throw std::runtime_error("unknown primitive type");
}

SimplexCollection
faces_single_dimension(const Mesh& mesh, const Simplex& simplex, const PrimitiveType face_type)
{
    SimplexCollection collection(mesh);

    faces_single_dimension(collection, simplex, face_type);

    return collection;
}

void faces_single_dimension(
    SimplexCollection& simplex_collection,
    const Simplex& simplex,
    const PrimitiveType face_type)
{
    simplex_collection.add(
        face_type,
        faces_single_dimension_tuples(simplex_collection.mesh(), simplex, face_type));
}

std::vector<Tuple> faces_single_dimension_tuples(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType face_type)
{
    assert(simplex.primitive_type() <= mesh.top_simplex_type());
    assert(face_type <= mesh.top_simplex_type());
    switch (face_type) {
    case PrimitiveType::Vertex: return vertices(mesh, simplex); break;
    case PrimitiveType::Edge: return edges(mesh, simplex); break;
    case PrimitiveType::Triangle: return faces(mesh, simplex); break;
    case PrimitiveType::Tetrahedron: break;
    default: assert(false); // "unknown primitive type"
    }

    return {};
}

} // namespace wmtk::simplex
