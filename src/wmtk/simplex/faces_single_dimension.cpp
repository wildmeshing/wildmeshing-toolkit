#include "vertices.hpp"

namespace wmtk::simplex {
std::vector<Tuple> vertices(const Mesh& mesh, const Simplex& simplex)
{
    const Tuple v0 = simplex.tuple();

    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        return {v0};
    }
    case PrimitiveType::Edge: {
        const Tuple v1 = mesh.switch_vertex(v0);
        return {v0, v1};
    }
    case PrimitiveType::Face: {
        const Tuple v1 = mesh.switch_vertex(v0);
        const Tuple v2 = mesh.switch_vertex(mesh.switch_edge(v0));
        return {v0, v1, v2};
    }
    case PrimitiveType::Tetrahedron: {
        const Tuple v1 = mesh.switch_vertex(v0);
        const Tuple v2 = mesh.switch_vertex(mesh.switch_edge(v0));
        const Tuple v3 = mesh.switch_vertex(mesh.switch_edge(mesh.switch_face(v0)));
        return {v0, v1, v2, v3};
    }
    default: throw std::runtime_error("unknown primitive type"); break;
    }
}
} // namespace wmtk::simplex