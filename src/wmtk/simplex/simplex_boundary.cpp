#include "simplex_boundary.hpp"

namespace wmtk::simplex {

SimplexCollection
simplex_boundary(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    const Mesh& m = mesh;
    const Tuple& t = simplex.tuple();

    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        // vertex does not have a boundary
        break;
    }
    case PrimitiveType::Edge: {
        collection.add(Simplex::vertex(t));
        collection.add(Simplex::vertex(m.switch_vertex(t)));
        break;
    }
    case PrimitiveType::Face: {
        const Simplex e0 = Simplex::edge(t);
        const Simplex e1 = Simplex::edge(m.switch_edge(t));
        const Simplex e2 = Simplex::edge(m.switch_edge(m.switch_vertex(t)));
        collection.add(e0);
        collection.add(simplex_boundary(m, e0, false));
        collection.add(e1);
        collection.add(simplex_boundary(m, e1, false));
        collection.add(e2);
        collection.add(simplex_boundary(m, e2, false));
        break;
    }
    case PrimitiveType::Tetrahedron: {
        const Simplex f0 = Simplex::face(t);
        const Simplex f1 = Simplex::face(m.switch_face(t));
        const Simplex f2 = Simplex::face(m.switch_face(m.switch_edge(t)));
        const Simplex f3 = Simplex::face(m.switch_face(m.switch_edge(m.switch_vertex(t))));
        collection.add(f0);
        collection.add(simplex_boundary(m, f0, false));
        collection.add(f1);
        collection.add(simplex_boundary(m, f1, false));
        collection.add(f2);
        collection.add(simplex_boundary(m, f2, false));
        collection.add(f3);
        collection.add(simplex_boundary(m, f3, false));
        break;
    }
    default: assert(false); break;
    }

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex