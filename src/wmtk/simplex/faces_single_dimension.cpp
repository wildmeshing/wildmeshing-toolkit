#include "faces_single_dimension.hpp"

namespace wmtk::simplex {
std::vector<Tuple> vertices(const Mesh& mesh, const Simplex& simplex)
{
    if (simplex.primitive_type() == PrimitiveType::Vertex) {
        return {};
    }

    const Tuple v0 = simplex.tuple();
    const Tuple v1 = mesh.switch_edge(mesh.switch_vertex(v0));

    if (simplex.primitive_type() == PrimitiveType::Edge) {
        return {v0, v1};
    }

    const Tuple v2 = mesh.switch_vertex(mesh.switch_edge(v0));

    if (simplex.primitive_type() == PrimitiveType::Face) {
        return {v0, v1, v2};
    }

    const Tuple v3 = mesh.switch_vertex(mesh.switch_edge(mesh.switch_face(v0)));

    if (simplex.primitive_type() == PrimitiveType::Tetrahedron) {
        return {v0, v1, v2, v3};
    }

    throw std::runtime_error("unknown primitive type");
}

std::vector<Tuple> edges(const Mesh& mesh, const Simplex& simplex)
{
    if (simplex.primitive_type() == PrimitiveType::Vertex ||
        simplex.primitive_type() == PrimitiveType::Edge) {
        return {};
    }

    const Tuple e0 = simplex.tuple();
    const Tuple e1 = mesh.switch_edge(mesh.switch_vertex(e0));
    const Tuple e2 = mesh.switch_vertex(mesh.switch_edge(e0));

    if (simplex.primitive_type() == PrimitiveType::Face) {
        return {e0, e1, e2};
    }

    const Tuple e3 = mesh.switch_edge(mesh.switch_face(e0));
    const Tuple e4 = mesh.switch_edge(mesh.switch_face(e1));
    const Tuple e5 = mesh.switch_edge(mesh.switch_face(e2));

    if (simplex.primitive_type() == PrimitiveType::Tetrahedron) {
        return {e0, e1, e2, e3, e4, e5};
    }

    throw std::runtime_error("unknown primitive type");
}

std::vector<Tuple> faces(const Mesh& mesh, const Simplex& simplex)
{
    return {};
}

std::vector<Tuple>
faces_single_dimension(const Mesh& mesh, const Simplex& simplex, const PrimitiveType face_type)
{
    if (face_type != PrimitiveType::Vertex) {
        throw std::runtime_error("No implementation for anything but Vertex.");
    }

    switch (face_type) {
    case PrimitiveType::Vertex: return vertices(mesh, simplex);
    case PrimitiveType::Edge: return edges(mesh, simplex);
    case PrimitiveType::Face: return faces(mesh, simplex);
    default: throw std::runtime_error("unknown primitive type"); break;
    }
}
} // namespace wmtk::simplex